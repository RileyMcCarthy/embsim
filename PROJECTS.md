# embsim projects — a system in a file

embsim runs one `System`: boards, bench components, the harness wires between
them, and a scenario. A **project** is that system written down, in a TOML
file, and the `embsim` command takes a netlist to a running system through
one: `survey` says what the netlist asks for, `new` writes a starter project,
`check` builds it, `run` runs it. Rust code loads the same file with
`embsim_board::Project`. This document is the guide to both: what a project
is, the workflow, the kinds the standard catalog ships, wiring boards to each
other, adding kinds of your own, the rules a project cannot break
([`DESIGN.md`](DESIGN.md) holds the rules for the whole of embsim), and how a
project's own crates extend the command (section 10).

## 1. What a project is

A project holds four lists, and each one is part of the `System` it builds:

| In the file | In the `System` | What it is |
|---|---|---|
| `[[board]]`, each with its `[[board.model]]` entries | `System::board` | a board: a netlist, either a KiCad export or one a catalog board kind bundles, built with a part registry, and the model each part the registry cannot place takes |
| `[[component]]` | `System::component` | a bench component: a part with pins and no board |
| `[[wire]]`, `[[mate]]` | `System::harness` | the harness: each wire joins two endpoints, or, with `volts`, sources one; each mate joins two connectors pin for pin |
| `[[switch]]`, `[[jumper]]`, `[[pin_short]]` | `System::scenario` | the scenario: switch poles and jumpers opened or closed, and two part pins shorted |

The file names **kinds**. A catalog (`embsim_board::Catalog`) turns each
kind into what it is: a board's netlist and part registry, a part model
registered into that registry, a core for the P2's package, or a bench
component. The standard catalog is `embsim_boards::catalog::StandardCatalog`.
Catalogs compose in a set (`embsim_boards::catalog::CatalogSet`), which
starts with the standard catalog and answers each kind from the catalog that
provides it. The `embsim` command builds every project with the set
`embsim_cli::shipped()`: the standard catalog with QEMU as a core the P2 can
hold (section 5). A project's own kinds join a set the same way (sections 7
and 10). The file itself holds no behaviour.
Every number a model uses stays in the model, with its citation. The file
says only which model sits where, which of the tables a model already
offers it takes, and what the bench does: the harness and the scenario.

From Rust, a project is two calls. `Project::load` reads the file, and
`Project::instantiate` returns the `System`, its boards built, not yet
started. The
example below runs as a doc test of `embsim-boards`, so its path is relative
to that crate's directory:

```rust
use embsim_board::Project;
use embsim_boards::catalog::StandardCatalog;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let project = Project::load("projects/header-pair.toml")?;
    // The checklist for one board, surveyed with the registry it builds with.
    let survey = project.survey(&StandardCatalog, "LEFT")?;
    assert!(survey.compliant());
    // The system the file describes; `build` analyzes it without starting it.
    let built = project.instantiate(&StandardCatalog)?.build()?;
    assert!(built.names_are_merged("LEFT.SIG", "RIGHT.SIG"));
    Ok(())
}
```

A test that runs a system live starts it the way
[`boards/tests/board_to_board.rs`](boards/tests/board_to_board.rs) does:
the clock stepped, the system started with `hold_time()`, the test's thread a
registered clock actor, and every read after a virtual settle
([`TESTING.md`](TESTING.md) rule 9). The `System` a project returns can take
more before it starts. That test adds a bench component and a harness wire of
its own with `System::component` and `System::harness`.

## 2. The file

Every table and key a project can hold is in the table below. A key the
format does not have is refused, and the error names it and the keys that
table takes:

```text
error: bad.toml: project does not parse: TOML parse error at line 5, column 1
  |
5 | colour = "red"
  | ^^^^^^
unknown field `colour`, expected one of `name`, `kind`, `netlist`, `model`
```

| Table | Key | What it says |
|---|---|---|
| `[[board]]` | `name` | the board's name in the system: the first word of every endpoint and net on it (`EC32.J203.41`, `EC32.Common_VDD`) |
| | `kind` | `"netlist"`, or a board kind the catalog ships (section 5) |
| | `netlist` | for `kind = "netlist"` only: the KiCad netlist export, relative to the project file |
| `[[board.model]]` | `part`, `mpn` or `value`, exactly one | the key: every part on the board whose symbol's part name, manufacturer part number or value is this string takes the model |
| | `kind` | the part kind (section 5) |
| `[board.model.options]` | depends on the kind | the kind's options; one the kind does not take is refused, naming the ones it does |
| `[[component]]` | `name`, `kind` | a bench component and its catalog kind (section 5); endpoints on it are `Name.Pin` |
| `[component.options]` | depends on the kind | the kind's options, taken and refused as `[board.model.options]` are |
| `[[wire]]` | `from`, `to` | two endpoints; a board's is `Board.Connector.Pin` |
| | `volts` | optional: `from` becomes a source at this voltage (section 6); one `from` takes one `volts` |
| `[[mate]]` | `a`, `b` | two connectors, `Board.Connector`: each pin of `a` joins `b`'s pin of the same number, and `b` may have more pins (section 6) |
| | `map` | optional: `[["a pin", "b pin"], …]`, the pairs a cable joins when it does not join them by number; only these are joined |
| `[[switch]]` | `part`, `pole`, `state` | a switch pole: `part = "EC32.S301"`, the pole numbered from 0 in the order the part declares its poles, `state = "open"` or `"closed"` |
| `[[jumper]]` | `part`, `state` | a two-pad jumper (a `Jumper*` or `SolderJumper*` symbol), open or closed; a three-pad one is two switch poles (pads 1–2 and 2–3) |
| `[[pin_short]]` | `a`, `b` | two part pins, `Board.Ref.Pin`, joined as one net: a scenario fault or a bodge wire |

A board or component name is not empty and has no dot or space, and no two
are the same. No two `[[board.model]]` entries on a board have the same key,
whatever field each names it by: the registry looks every key up in one
table, so `part = "X"` and `value = "X"` are one key. A path in the file (`netlist`, and the `image` and `rom`
options) is relative to the project file's directory. `Project::parse` reads
a project from text instead, with paths relative to the current directory
unless `Project::relative_to` gives another.

## 3. From a netlist to a running system

### The command

```bash
cargo install --path cli          # installs `embsim` in Cargo's bin directory
cargo run -p embsim-cli -- --help # or run it in place, from the workspace
```

It has four subcommands:

```bash
embsim survey board.net                              # the checklist
embsim new board.net --name BOARD -o board.toml      # a starter project
embsim check board.toml                              # build it, time held
embsim run board.toml --for 20ms --net BOARD.NET     # run it
```

The walk-through below takes the DS2 force-gauge add-on from its KiCad
export (`board/tests/fixtures/ds2_addon.net`) to a running system, in a
directory of its own made from the workspace root:

```bash
mkdir ds2 && cp board/tests/fixtures/ds2_addon.net ds2/ && cd ds2
```

### Step 1: `embsim survey`, the checklist

```bash
embsim survey ds2_addon.net
```

```text
ds2_addon.net: 31 parts
  30 populated by the catalog
  1 need a model
  0 placed with a pin table the netlist does not use
  0 refused
  5 connectors

populated by class (the symbol or reference designator says what it is):
  capacitor     12  C1 C2 C3 C4 C5 C6 C7 C8 C9 C10 C11 C12
  connector      5  J1 J2 J3 J4 J5
  jumper         4  JP1 JP2 JP3 JP4
  resistor       9  R1 R2 R3 R4 R5 R6 R7 R8 R9

need a model:
  U1  part "ADS122U04"  value "ADS122U04"
      16 pins: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16
      could be: ads122u04 (part number ADS122U04IPW)

connectors (a wire's board end is Board.Connector.Pin):
  J1  value "MCU"  5 pins
    pin  name   net
    1    Pin_1  +3V3
    2    Pin_2  GND
    3    Pin_3  Net-(J1-Pin_3)
    4    Pin_4  Net-(J1-Pin_4)
    5    Pin_5  Net-(J1-Pin_5)
  J2  value "Analog1"  4 pins
    pin  name   net
    1    Pin_1  VDDA
    2    Pin_2  VSS
    3    Pin_3  A0
    4    Pin_4  A1
  …
```

The survey classifies every part through the registry the board will build
with (section 4) and sorts the parts into these lists:

- **Populated by class.** The part's symbol, or its reference designator,
  says what it is.
- **Populated by a catalog model.** The catalog placed the part by a key it
  carries, and the survey shows the key.
- **Need a model.** A part the catalog cannot place is listed with the keys a
  `[[board.model]]` can name (its part name, value and manufacturer part
  number), its pins, and what it could be:
  - **The kinds its keys name.** A kind fits by part number first: compared
    on their letters and digits, one of the part's keys and a number the kind
    is for are the same, or one starts with the other, as `ADS122U04` starts
    `ADS122U04IPW`. Failing that, a kind fits by part family: a key contains
    the family the kind's model is for, as `TG2520SMN 26.0000M-ECGNNM3`
    contains `TG2520SMN` (section 5, the "seats on" column). A key with fewer
    than five letters and digits names no part. The survey shows only the
    stronger kind of fit any kind makes.
  - **Else, the kinds without a model the board allows** (section 3, step
    3): `switch`, `boundary` or `mechanical`, each with what allows it, the
    part's designator, symbol, name or nets.
  - **Else, that it needs a model**, written for it (section 7).

  Pins name no kind. An EDA export numbers every package's pins from 1, so
  two parts with as many pins share a table whatever each is. A part whose
  symbol names one part and whose manufacturer part number names another of
  its family (the Edge board's `U25`: `AM26LV32xD` and `AM26LS32CD`) is
  flagged: a model is one part's, and the netlist does not say which the
  board carries.
- **Placed with a pin table the netlist does not use.** The model's pins and
  the netlist's pins are shown side by side, with the table that fits.
- **Refused.** A class the part cannot be, such as a resistor with three
  pins.
- **Connectors.** Every connector pin is listed with its name and its net,
  because those pins are where a wire may land.

### Step 2: `embsim new`, the starter project

```bash
embsim new ds2_addon.net --name DS2 -o ds2.toml
```

```text
wrote ds2.toml
  board DS2: 31 parts, 30 populated by the catalog, 0 pin tables chosen
  1 model stub to fill in, for 1 part that need a model
  5 connectors to wire; then `embsim check ds2.toml`
```

The file answers the checklist as far as the catalog can:

- **The board.** It is `kind = "netlist"`, with the netlist relative to the
  project file. When the two share nothing but the filesystem root, the path
  is written whole.
- **Pin tables.** For each part the catalog placed with a table the netlist
  does not use, the file has a `[[board.model]]` choosing the one that fits,
  or, where no table of the model fits, a comment saying so.
- **Model stubs.** Each part that needs a model gets a commented stub, keyed
  by its manufacturer part number, else its part name, else its value, under
  the survey's sentence for it. When exactly one kind fits by part number,
  and the part's symbol and number agree, the stub names that kind.
  It also carries the options the kind cannot go without, each at an example
  value, and a `pins` line when the table that fits is not the default.
- **Endpoints.** Every connector pin is listed as the endpoint a wire names.

Without `-o` the project goes to standard output. `--name` defaults to the
netlist's file name, with its dots and spaces made underscores. `new` does
not replace a file that exists unless it is given `--force`. The stub and the
first connector in `ds2.toml`:

```toml
# U1: part "ADS122U04", value "ADS122U04"; 16 pins: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
# 15, 16.
# Could be: ads122u04 (part number ADS122U04IPW).
# [[board.model]]
# part = "ADS122U04"
# kind = "ads122u04"

# ---- Connectors: where a wire may land -------------------------------------
# …
# J1 "MCU", 5 pins:
#   endpoint  name   net
#   DS2.J1.1  Pin_1  +3V3
#   DS2.J1.2  Pin_2  GND
#   DS2.J1.3  Pin_3  Net-(J1-Pin_3)
#   DS2.J1.4  Pin_4  Net-(J1-Pin_4)
#   DS2.J1.5  Pin_5  Net-(J1-Pin_5)
```

Checked as written, the project is refused with the survey as the error:

```bash
embsim check ds2.toml
```

```text
project ds2.toml
  board DS2 (netlist): 31 parts: 30 classified, 1 need a model, 0 with pins the netlist does not have, 0 refused, 5 connectors
error: board DS2 is not ready to build:
31 parts: 30 classified, 1 need a model, 0 with pins the netlist does not have, 0 refused, 5 connectors
needs a model:
  U1  part "ADS122U04"  value "ADS122U04"  (16 pins)
connectors:
  …
give each part that needs a model a [[board.model]] with its part, mpn or value and the kind it is; a kind seats only on a part that is what the kind says, and a part no kind is for needs a model written for it (PROJECTS.md §7); the part kinds are "p2", "tg2520smn", …
```

### Step 3: give each part its model

Uncomment each stub and give it the kind that fits. U1 is the ADS122U04,
by its symbol's part name:

```toml
[[board.model]]
part = "ADS122U04"
kind = "ads122u04"
```

A netlist transcribed from a schematic usually names pins by function where
an EDA export numbers them, and the catalog places a part by its number with
the datasheet's numbered table. The P2-EC32MB's netlist is one of these
(`embsim survey boards/netlists/p2_ec32mb.net`, from the workspace root), and
its survey lists 19 parts like this:

```text
placed with a pin table the netlist does not use:
  U101, U601  mpn "74LVC2G04GW,125": 74lvc2g04, pins = "sot363"
      declares 1, 2, 3, 4, 5, 6
      the netlist has 1A, 1Y, 2A, 2Y, GND, VCC
      pins = "by-function" declares the netlist's pins
```

`embsim new` writes the entry that chooses the table:

```toml
[[board.model]]
mpn = "74LVC2G04GW,125"
kind = "74lvc2g04"
[board.model.options]
pins = "by-function"
```

Choose a kind by what the part is, and a table by the netlist's pins. A
kind seats only on a part it is (section 8): a model's kind on a part whose
part name, manufacturer part number or value contains the family the model
is for, whatever the part's pins.

Three kinds say what a part is without a model, and each seats only where the
board says the part is one:

- `switch` pairs the part's pins into poles, for a part whose designator is
  `S`, `SW`, `JP` or `SJ`, whose symbol is a `SW_…`, or whose symbol name or
  value says switch, jumper or solder link (the P2-EC32MB's `J101`, "Solder
  Link Pads").
- `boundary` makes a part a connector, for one whose designator is `J`, `P`
  or `CN` or whose symbol is a `Conn…`.
- `mechanical` says the part has nothing electrical (a PCB line, a layout
  node, a mounting hole), for one whose pins sit on one net at most. A part
  whose pins join two nets carries current between them, and that is
  behaviour. The build also reports a mechanical pad on a net a pin drives
  (`MechanicalOnDrivenNet`).

When no kind fits, the part needs a model. It may exist outside the catalog
(section 9 lists the parts the catalog does not model yet, and where their
models are) or need writing (section 7). Until it has one the board does not
build: there is no stand-in.

### Step 4: wire the connectors

A board on its own has no supply: ground and every input supply are the
harness's (DESIGN.md rule 6: ground is a declared terminal). The add-on
generates none of its rails. It takes 3.3 V and ground on its digital
connector `J1`, and on its analog connector `J2`:

```toml
[[wire]]
from = "BENCH.3V3"
to = "DS2.J1.1"
volts = 3.3

[[wire]]
from = "BENCH.GND"
to = "DS2.J1.2"
volts = 0.0

[[wire]]
from = "BENCH.3V3A"
to = "DS2.J2.1"
volts = 3.3

[[wire]]
from = "BENCH.GNDA"
to = "DS2.J2.2"
volts = 0.0
```

`BENCH` is no board or component in the project. A wire with `volts` makes
the endpoint in its `from` a supply of its own, and any other wire may then
join that supply. Section 6 has the rules for where a wire may land.

### Step 5: `embsim check`

```bash
embsim check ds2.toml
```

```text
project ds2.toml
  board DS2 (netlist): 31 parts: 31 classified, 0 need a model, 0 with pins the netlist does not have, 0 refused, 5 connectors
  1 board, 0 bench components, 4 wires, 0 mates
build findings (11), the system before its first wake:
  FloatingSense { net: "DS2.GPIO1", kind: Digital }
  FloatingSense { net: "DS2.GPIO0", kind: Digital }
  FloatingSense { net: "DS2.~RESET", kind: Digital }
  …
  FloatingSense { net: "DS2.AIN0", kind: Analog }
ok: ds2.toml builds
```

`check` loads the project, surveys each board, builds the system and starts
it with virtual time held. Every part is attached and every attach-time
drive resolved, but no wake has fired. `check` prints what the build found
and exits 0. Any refusal makes it exit 1, with the text that says what to
fix. The findings are the netlist's own. Among them:

- `GPIO0` and `GPIO1` come out on `J3`, which nothing is wired to here.
- The converter's `~RESET` is on a net no other pin shares. The add-on's own
  tests hold that as a finding about the board
  (`boards/tests/ds2_project.rs`).

### Step 6: `embsim run`

```bash
embsim run ds2.toml --for 5ms --net DS2.+3V3 --net DS2.VDDA --net DS2.~RESET
```

```text
project ds2.toml
  catalogs: embsim-boards, embsim-p2-qemu
  …
running for 5.000000 ms of virtual time
findings at build, before any wake (11):
  FloatingSense { net: "DS2.GPIO1", kind: Digital }
  …
ran 5.000000 ms of virtual time in 0.001 s
net DS2.+3V3: Analog(3.3)
net DS2.VDDA: Analog(3.3)
net DS2.~RESET: Floating
findings: 11 (11 at build, 0 while running)
at 5.000000 ms, each finding's net read again:
  no longer true (0):
  still true (11):
    FloatingSense { net: "DS2.GPIO1", kind: Digital }
    …
  about the board as built and wired (0):
```

`run` starts the system on a stepped virtual clock and runs it for `--for`
of virtual time. The duration is a number and a unit: `ns`, `us` (or `µs`),
`ms` or `s`. It prints the report in three parts:

- **At build, before any wake.** The findings the build made, the system
  before its first wake: every rail with a soft-start is down then, so a
  board with regulators lists its rails as unsourced here.
- **While it runs.** `run` looks at the system every 100 µs of virtual time.
  It prints each new finding stamped with the instant of the look that found
  it, and what the parts a catalog built report, under their name: a P2's
  package the instant its core started, a QEMU core its console output
  (section 5), a host serial port its path (the first look, at 0).
- **At the end.** What each part that reports says of itself (a P2's core
  and how it stands, a host port's bytes each way), the nets named with
  `--net` (`Board.Net`, as the board's
  netlist spells the net; a net the system does not have is refused before
  the run starts), then every finding again, its net read once more. A
  finding is about the system at an instant, and findings are not withdrawn
  as a run goes on, so this is where a build finding the run cleared is
  said to be: "no longer true", with what its net reads now (a rail that
  came up reads its voltage). A floating sense, an unsourced power net and a
  down rail hold while their net floats; a fight holds while its net reads
  one. A finding with no net of its own to read again (an undecoupled
  supply pin, an open drain with no pull-up) is listed as about the board
  as built and wired.

On the P2-EC32MB project (`boards/projects/ec32-netlist.toml`), run for
10 ms, the build lists the module's ten rails as unsourced, and the end lists
each as no longer true at the voltage its regulator holds:

```text
  no longer true (18):
    …
    PowerNetUnsourced { net: "EC32.Common_VDD" }: EC32.Common_VDD reads Analog(1.8133333333333335)
    PowerNetUnsourced { net: "EC32.VIO_00_07" }: EC32.VIO_00_07 reads Analog(3.3)
    …
```

Virtual time advances only to the next instant something happens, so two
runs of a project print the same report, apart from the line with the wall
time (and, with a host serial port, when the host's bytes landed: section 5).
Without `--for`, a run lasts until it is interrupted (Ctrl-C, or SIGTERM).
Either way, the first interrupt ends the run at its next look: it prints
`interrupted at` the instant it reached, then the summary a run that reaches
its `--for` prints. A second interrupt ends the process at once.

## 4. How a board is populated

Every board is built by `Board::from_netlist` with a `PartRegistry`, and
surveyed with that same registry first. A part gets its class in this order:

1. **By its symbol.** The registry's first tier classifies a part by its
   symbol's part name (its libsource):
   - resistors, capacitors and inductors;
   - connectors (`Conn*`, `Screw_Terminal*`);
   - jumpers (`Jumper*`, `SolderJumper*`);
   - two-pin `SW_*` switches;
   - test points;
   - mounting holes, logos and fiducials.

   A netlist with no libsource at all, such as one transcribed from a PDF,
   classifies these by reference designator instead: `R`, `C`, `L`, `D`,
   `J` or `P` for a connector, `TP` for a test point, `H` or `MK` for a
   mounting hole. A `kind = "netlist"` board turns this on. A diode or LED
   takes its forward drop from the part it is, so a diode symbol or a `D`
   designator is not enough: the part is placed only by a registry entry for
   its part number or name, which the element library has for the parts it
   knows.
2. **By part number, from the catalog's base registry.** A
   `kind = "netlist"` board starts from this registry. It holds the element
   library (`embsim_models::pwl_library`: diodes, LEDs, FETs, transistors
   and current regulators, keyed by part number), and every model in section
   5's table under the part numbers in its "placed by part number" column,
   with its default pin table. Two kinds are never placed by number: the
   processor, because what runs inside it is the thing under test, and an
   SD card, because the card in the socket is too. A catalog board kind
   builds with its own registry instead, which places every part of that
   board but the ones the project chooses.
3. **By a `[[board.model]]`.** The kind is registered into the board's
   registry under the entry's key. The registry looks a part up by its part
   name first, then its manufacturer part number, then its value. An entry
   is refused, with the reason, when:
   - it matches no part;
   - another entry on the board has the same key, by any field;
   - a part it reaches is not what the kind says it is: its keys do not
     contain the family the model is for, or the board does not say it is a
     switch, a connector or a part with nothing electrical (section 3, step
     3);
   - another key of the same part comes first (the error names the key to
     use instead);
   - the part's symbol already makes it a resistor, a connector or another
     class a model cannot change.

   A key is never a reference designator. A model belongs to a kind of part,
   and every part the key reaches is that model.

An entry does override a class the reference designator only guessed. On
the P2-EC32MB's transcribed netlist, `J101` (a solder link) and `J701`/`J702`
(mounting holes) are `J` parts that are not connectors. The project in
`boards/projects/ec32-netlist.toml` makes them a one-pole `switch` and
`mechanical` by their values.

A part nothing classifies stops the build (DESIGN.md rule 1). The survey is
the error, and it lists every such part with the keys an entry can name.

## 5. The kinds the standard catalog ships

The two tables below are generated from the catalog: the test
`projects_md_tabulates_every_kind_the_catalog_ships`
(`boards/src/catalog.rs`) builds them from the catalog, reading each
option off the kind's own registration, and fails when this file differs by
a character. When the catalog changes, the failure prints the table to paste
here.

### Board kinds

<!-- board-kinds:begin -->
| kind | what it is |
|---|---|
| `netlist` | any board, from its KiCad netlist export: `netlist = "board.net"`, relative to the project file; it starts from the base registry |
| `p2-ec32mb` | the Parallax P2-EC32MB module from its bundled netlist, every part placed but the processor, `U100` |
<!-- board-kinds:end -->

### Part kinds

<!-- part-kinds:begin -->
| kind | model | seats on | placed by part number | `pins` (the first is the default) | other options |
|---|---|---|---|---|---|
| `p2` | the Propeller 2 package | a part whose part name, mpn or value contains P2X8C4M64P | never (it is for `P2X8C4M64P`) | fixed: `P2X8C4M64P` (86 pins) | `core` = `"held-in-reset"` (required) |
| `tg2520smn` | EPSON TCXO; frequency from the part's value or number | a part whose part name, mpn or value contains TG2520SMN | `TG2520SMN 20.0000M-ECGNNM3` | `"numbered"` (4 pins), `"by-function"` (4 pins) | — |
| `74lvc2g04` | NXP dual inverter | a part whose part name, mpn or value contains 74LVC2G04 | `74LVC2G04GW,125` | `"sot363"` (6 pins), `"by-function"` (6 pins) | — |
| `sn74lvc1g14` | TI Schmitt inverter | a part whose part name, mpn or value contains 74LVC1G14 | `SN74LVC1G14DBVR`, `SN74LVC1G14DBVT` | `"sot23"` (5 pins) | — |
| `aps6404l` | AP Memory PSRAM | a part whose part name, mpn or value contains APS6404L | `APS6404L-3SQR-ZR` | `"sop8"` (8 pins), `"by-function"` (9 pins) | — |
| `w25q128jv` | Winbond serial NOR flash, blank or holding an image | a part whose part name, mpn or value contains W25Q128JV | `W25Q128JVSIM`, `W25Q128JVSIM TR`, `W25Q128JVSIQ` | `"soic8"` (8 pins), `"by-function"` (8 pins), `"spi-only"` (4 pins) | `id` = `"im"`, `"iq"`; `image = "boot.bin"` — a file the part holds from address 0, the rest erased, relative to the project file |
| `sd-card` | a card in an SD socket | a connector: designator J, P or CN, or a Conn… symbol | — | `"microsd"` (8 pins), `"by-function"` (8 pins), `"spi-only"` (4 pins) | `image = "card.img"` — the card in the socket: a card image file, relative to the project file (required) |
| `ap62301` | Diodes buck; setpoint from its feedback divider | a part whose part name, mpn or value contains AP62301 | `AP62301Z6-7` | `"sot563"` (6 pins), `"by-function"` (5 pins) | — |
| `ncp114` | onsemi LDO; setpoint from the part's value or number | a part whose part name, mpn or value contains NCP114 | `NCP114AMX330TCG` | `"udfn4"` (4 pins), `"by-function"` (5 pins) | — |
| `xl1509` | XLSEMI buck; version from the part's value or number | a part whose part name, mpn or value contains XL1509 | `XL1509-3.3E1`, `XL1509-5.0E1`, `XL1509-12E1` | `"sop8"` (8 pins) | — |
| `ucc12040` | TI isolated DC/DC; setpoint from its SEL strap | a part whose part name, mpn or value contains UCC12040 | `UCC12040DVE`, `UCC12040DVER` | `"soic16"` (16 pins) | — |
| `stm1061` | ST voltage detector, from its ordering code | a part whose part name, mpn or value contains STM1061 | `STM1061N16WX6F` | `"sot23"` (3 pins), `"by-function"` (3 pins) | — |
| `6n137` | Lite-On optocoupler | a part whose part name, mpn or value contains 6N137 | `6N137` | fixed: `6N137` (7 pins) | — |
| `vo2631` | Vishay dual optocoupler | a part whose part name, mpn or value contains VO2631 | `VO2631` | fixed: `VO2631` (8 pins) | — |
| `iso67xx` | TI digital isolator, the member the key names | a part whose part name, mpn or value contains ISO6720, ISO6721, ISO6731, ISO6740, ISO6741 or ISO6742 | `ISO6721BDR`, `ISO6731DWR`, `ISO6740DWR`, `ISO6740FDWR`, `ISO6741DWR`, `ISO6742DWR` | fixed, the member's: `ISO6721BDR` (8 pins), `ISO6731DWR` (16 pins), `ISO6740DWR` (16 pins), `ISO6740FDWR` (16 pins), `ISO6741DWR` (16 pins), `ISO6742DWR` (16 pins) | — |
| `ads122u04` | TI 24-bit ADC, as it comes out of reset | a part whose part name, mpn or value contains ADS122U04 | `ADS122U04IPW`, `ADS122U04IPWR` | `"tssop16"` (16 pins) | — |
| `switch` | a switch whose poles pair the part's pins, each open | a switch or jumper: designator S, SW, JP or SJ, a SW_… symbol, or a name that says switch, jumper or solder link | — | the part's own | `poles = [["1", "2"]]` — the part's pins paired into poles, each open until a [[switch]] closes it (required) |
| `mechanical` | a part with pads and nothing electrical | a part whose pads sit on one net at most | — | the part's own | — |
| `boundary` | a connector, by its symbol's part name | a connector: designator J, P or CN, or a Conn… symbol | — | the part's own | — |
<!-- part-kinds:end -->

**Seats on.** A kind seats only on a part that is what it says, and an entry
that reaches any other part is refused, naming the part and what the kind is
for (section 8). A model's kind is for a part family: a part whose part
name, manufacturer part number or value contains it, compared on letters
and digits, so `SN74LVC1G14DBVR` is a `74LVC1G14` and
`TG2520SMN 26.0000M-ECGNNM3` a `TG2520SMN`. The model then reads what it
needs from the part, or refuses it: the oscillator reads its frequency from
the value. The `sd-card` kind sits in a socket, so it seats on a connector.
`switch`, `boundary` and `mechanical` seat where the board says the part is
one (section 3, step 3). A part's pins say nothing here: a pin table only
has to match once the kind has seated.

**Pin tables.** `pins` picks the table of pins the model declares, and the
survey compares it with the netlist's pins as sets, in both directions. The
default is the datasheet's numbered table, which is how an EDA export names
pins. `by-function` is the table a netlist transcribed from a schematic
uses. `spi-only` is a flash's or a card's four SPI signals alone, for a
four-wire bench. A kind whose table is fixed takes its model's one table;
the ISO67xx family's is the member's. A kind with "the part's own" takes the
part's pins, whatever they are.

**Options.** An option only chooses among what the model already offers: a
pin table, a JEDEC ID, the file a memory holds, what runs inside the
package. Where a model needs a number from the board, it reads it from the
netlist. The oscillator's frequency, the LDO's and the XL1509's output
voltage and the STM1061's threshold come from the part's value or part
number. The AP62301's setpoint comes from its feedback divider and the
UCC12040's from its `SEL` strap, both read from the netlist when the part
attaches. The ISO67xx member comes from the key it is assigned by. A kind
that reads its part's value or number checks every part its key reaches
when it is registered, so a part it cannot configure is an error naming the
part, before anything is built. Two parts that one key reaches and that
configure the model differently are refused: one key takes one model.

**`boundary`** makes a part a connector by its symbol's part name
(`part = "…"`), for a project-library connector symbol whose name the first
tier does not know. On a netlist with no part names, a `J` or `P` part is a
connector by its reference designator already.

### The P2's core, in the `embsim` command

The `p2` kind is the package, and its `core` option names what runs inside
it: a **core kind**, from whichever catalog in the set provides it. The
package is the same for every core (its START gate, its bank supplies, its
brownout hold), and the rest of the entry's options are the core's. The
standard catalog has one core, `"held-in-reset"`, the chip before it runs,
with its pads released; it takes no other option. The `embsim` command's
set adds `"qemu"` (`embsim_p2_qemu::catalog::QemuCores`), which boots the P2
on QEMU off whatever the board gives it. The boot ROM is Parallax's own
(`embsim_p2_qemu::BOOT_ROM`) unless the `rom` option names a file. On the
P2-EC32MB the chip boots from its flash, which the `w25q128jv` kind's
`image` option fills:

```toml
[[board]]
name = "EC32"
kind = "p2-ec32mb"

[[board.model]]
value = "P2X8C4M64P"
kind = "p2"
[board.model.options]
core = "qemu"

[[board.model]]
value = "SPI Flash 16MB (128Mb)"
kind = "w25q128jv"
[board.model.options]
pins = "by-function"
image = "boot.bin"

# … the carrier's five wires and S301 poles 1 and 3 closed, as in
# boards/projects/ec32-carrier.toml
```

```bash
EMBSIM_QEMU_P2_BUILD=/path/to/qemu/build-p2 cargo install --path cli
embsim run boot.toml --for 20ms --net EC32.Common_VDD
```

```text
…
[   5.600000 ms] EC32.U100: the core started at 5.500000 ms
[  10.000000 ms] EC32.U100: P62 "B"
ran 20.000000 ms of virtual time in 0.044 s
EC32.U100: core "qemu": started at 5.500000 ms
EC32.U100: QEMU: 16953 pad yields; running; console P62 "B"
net EC32.Common_VDD: Analog(1.8133333333333335)
```

The package reports its start and, at the end, its gate; the QEMU core
reports its console, per pad, and its yields. Both print under the part's
name.

QEMU is linked when the command is built (`EMBSIM_QEMU_P2_BUILD`; making the
QEMU tree is [`p2-qemu/README.md`](p2-qemu/README.md)). A build without it
refuses the entry and says how to link it:

```text
error: board EC32: [[board.model]] value = "P2X8C4M64P" (kind "p2"): embsim-p2-qemu was built without a QEMU tree; set EMBSIM_QEMU_P2_BUILD to a configured QEMU build with the p2 target and rebuild
```

QEMU is one machine per process, so a `core = "qemu"` key may reach only one
part. The chip boots when the board is built, which means `survey` never
boots it and `check` does, with time held. No command makes a flash image
yet. `embsim_p2_qemu::flashimage::boot_flash(STAGE1, &program)` lays out
stage-1 and a program, as the test that boots this project does
(`run_boots_the_p2_off_the_modules_flash`, `cli/tests/cli.rs`).

### Bench component kinds

A `[[component]]` is a part with pins and no board. Its pins are its
endpoints, `Name.Pin`, and its options are `[component.options]`. The
standard catalog ships two kinds.

**`host-serial`**: the host's end of a serial link, a PTY whose bytes are
levels on the wire (`embsim_board::HostPty::open_on_rail`). A host program
opens the PTY's path and reads and writes it as it would a serial adapter.

| Pin | What it is |
|---|---|
| `TX` | what the host sends, driven onto the wire: a high at `VIO` above `GND`, a low at `GND`, behind the push-pull default; released while `VIO` reads no voltage |
| `RX` | what the host receives: JESD8C.01's 0.8 V / 2.0 V pair against `GND`, the pair a 3.3 V LVCMOS input and a 5 V TTL input both take |
| `VIO` | the host's I/O rail, against `GND`: wire the host's real rail (a Raspberry Pi's 3.3 V) |
| `GND` | the host's ground |

The pins are named from the host's side, so a wire reads `HOST.TX` to the
board's receive pin.

| Option | What it says |
|---|---|
| `baud` | required: the link's rate, framed 8N1. A host names its rate, and the kind invents none |
| `path` | the path the PTY is reached at (a symlink to it), relative to the project file; by default `.embsim/<name>.pty` beside it |

`embsim run --pty PATH` sets `path` for the project's one `host-serial`,
relative to the current directory; `--pty NAME=PATH` sets it for the
component `NAME`, and is needed when there are several. The run prints the
path at its first look, before virtual time moves, so a host can open it,
and at the end the bytes that crossed each way:

```text
[   0.000000 ms] HOST: host serial at /tmp/tty.rpi, 2000000 baud 8N1
…
HOST: host serial at /tmp/tty.rpi: 412 bytes from the host, 9330 to it, 0 framing errors
HOST: the host wrote during the run: its bytes landed when it wrote them, so this run is reproducible in what the host sent, not in when
```

A host writes in wall time, so its bytes land at whatever virtual instant
the run has reached. A run whose host wrote anything says so at the end.

**`scripted-source`**: one pin, `OUT`, driven through a list of steps
(`embsim_board::ScriptedSource`). It is the smallest stimulus that lets a
project do something over time: press a button, brown out a rail, sweep a
sensor's output.

| Option | What it says |
|---|---|
| `ohms` | required: the source's output impedance, more than 0 Ω. A scenario line names it (DESIGN.md rule 6); an ideal constant supply is a `[[wire]]` with `volts` |
| `steps` | required: `[["1ms", 3.3], ["2ms", 0.0], …]`, each an instant (written as `--for` is) and the volts the pin drives from then on, behind `ohms`. Instants count from the instant the system starts and increase strictly; volts are in the engine's frame, as a wire's `volts` are |

Before its first instant the pin is released; after its last it holds.
Each step is one drive published at its instant, on a wake the source arms
for it (`boards/tests/scripted_source.rs` reads each step land at its own
nanosecond).

## 6. Board to board

A harness attaches to a board at its boundary. It is made of two tables:

- **`[[wire]]`** joins two endpoints. A wire's end on a board is a connector
  pin, `Board.Connector.Pin`. Its other end is a connector pin on the same
  board or another one, a bench component's pin (`Name.Pin`), or a supply.
- **`[[mate]]`** joins two connectors at once, `a` and `b`, each
  `Board.Connector`: a module seated in its socket, a header on a header, a
  cable between two boards.

A wire with `volts` makes its `from` a source at that voltage. That end is
usually a name no board or component has (`BENCH.5V`), and any other wire
may join it without `volts`. One name is one source: a second wire with
`volts` from the same `from` is refused, naming the first, because two
voltages on one name would be two sources fighting through whatever joins
them. A second source takes a name of its own.

A wire to a pin that is not on a connector is refused, and the error lists
the connectors:

```text
error: [[wire]] BENCH.GND to HDR.R1.2: R1 is not a connector, and a wire lands on a connector pin; HDR's connectors are J1
```

A `[[pin_short]]` is the one way to join two pins that are not on
connectors. It is a scenario line, for a fault or a bodge wire, and it may
join any two part pins on the boards.

### Mates

Without a `map`, a mate joins each pin of `a` to `b`'s pin of the same
number. Every pin of `a` has to land: a pin `b` lacks is refused, naming the
pins. `b` may have more pins than `a`, and those stay open, as the contacts
of a socket wider than the card seated in it do. So `a` is the side with
fewer pins: the module's fingers, the cable's plug.

A cable that does not join pins by number says which it joins with `map`, a
list of `["a pin", "b pin"]` pairs. Only those pairs are joined, so a pin a
cable does not wire stays open. A pin the connector lacks, or a pin named
twice on one side, is refused.

[`boards/projects/header-pair.toml`](boards/projects/header-pair.toml)
mates two boards. Each board is the header board (`header.net`), a two-pin
connector `J1` with a 10 kΩ resistor from its `SIG` to its `GND`. The file
mates the two `J1`s pin for pin and puts the bench's 0 V on the left one's
ground:

```toml
[[board]]
name = "LEFT"
kind = "netlist"
netlist = "header.net"

[[board]]
name = "RIGHT"
kind = "netlist"
netlist = "header.net"

[[mate]]
a = "LEFT.J1"
b = "RIGHT.J1"

[[wire]]
from = "BENCH.GND"
to = "LEFT.J1.2"
volts = 0.0
```

```bash
embsim check boards/projects/header-pair.toml
```

```text
project boards/projects/header-pair.toml
  board LEFT (netlist): 2 parts: 2 classified, 0 need a model, 0 with pins the netlist does not have, 0 refused, 1 connectors
  board RIGHT (netlist): 2 parts: 2 classified, 0 need a model, 0 with pins the netlist does not have, 0 refused, 1 connectors
  2 boards, 0 bench components, 1 wire, 1 mate
build findings: none
ok: boards/projects/header-pair.toml builds
```

Each board keeps its own net names. The mate makes `LEFT.SIG` and
`RIGHT.SIG` one node, and `BuiltSystem::names_are_merged` says so (the
first example in section 1).
[`boards/tests/board_to_board.rs`](boards/tests/board_to_board.rs) proves
it live. It drives a 25 Ω pad on `LEFT.J1.1`, and `RIGHT.SIG` reads
3.3 V divided between the pad and both boards' resistors in parallel.
[`boards/tests/mates.rs`](boards/tests/mates.rs) holds a crossed map and a
one-wire map to the pins they name.

### Which connectors mate, and how their pins map

Nothing in two netlists says which of their connectors mate: that is the
assembly, and the project says it. What the netlists do give is each
connector's pins, with their names and nets, and `embsim survey` lists
them:

- `embsim survey board.net` lists every connector of a netlist board with
  its pins, each pin's name and its net. A part the survey can make a
  connector (a `J`, `P` or `CN` designator, a `Conn…` symbol) is listed with
  the same table under "need a model", so the pins a mate will land on are
  in front of you before it has its `boundary` entry.
- `embsim survey --kind p2-ec32mb` lists a board kind the catalog ships the
  same way, surveyed with the registry a project builds it with.

Put the two lists side by side. Where the two connectors name each pin
alike (the same signal name on the same number, or the same net name),
they mate by number. Where they do not, the cable crosses, and the map says
how. Write down why in a comment beside the mate: it is the one place the
project records the assembly.

### Seating a module in a carrier, and wiring a cable

[`boards/projects/edge-ec32-ds2.toml`](boards/projects/edge-ec32-ds2.toml)
is the MaD machine's electronics: the MaD Edge carrier, the P2-EC32MB
module in its socket, and the DS2 force-gauge add-on on the carrier's force
cable.

**The module in its socket.** `embsim survey board/tests/fixtures/mad_edge.net`
lists the Edge board's socket, `J3`, with its 80 pins. Its symbol is the
board's own (`P2_EDGE_MODULE_SOCKET`), so the survey offers it `boundary`
by its `J`, and lists its pins:

```text
  J3  part "P2_EDGE_MODULE_SOCKET"  value "P2_EDGE_MODULE_SOCKET"  mpn "450-00309"
      80 pins:
        pin  name             net
        1    NC@1             unconnected-(J3-NC@1-Pad1)
        2    NC@2             unconnected-(J3-NC@2-Pad2)
        3    P37              P37
        …
        41   5V@1             +5V
        …
```

`embsim survey --kind p2-ec32mb` lists the module's card edge, `J203`,
with the 60 fingers its netlist declares:

```text
  J203  value "Edge Socket Pads"  60 pins
    pin  name             net
    1    NC               NC_Net
    2    NC               NC_Net
    3    P37              P2_IO37
    …
    41   5V               VIN_Edge
    …
```

Both netlists number the card edge by finger, and both label each finger
alike (`P37` on 3, `5V` on 41), so the module mates by number. The module's
netlist leaves out the 20 fingers it keeps for its PSRAMs (55, 56, 59–67 and
69–77: `P40`–`P57` and the `V40`/`V48` bank supplies), so `J203` has fewer
pins than `J3` and is `a`:

```toml
[[board.model]]          # under the EDGE board
part = "P2_EDGE_MODULE_SOCKET"
kind = "boundary"

[[mate]]
a = "EC32.J203"
b = "EDGE.J3"
```

**The force cable.** The carrier's `J9` ("Force", a six-way Molex 43045)
and the add-on's `J1` ("MCU", a five-way 2.54 mm header) share no pin names
and no net names:

```text
  J9  value "Force"  6 pins          J1  value "MCU"  5 pins
    1    Pin_1  …/IFG_5V               1    Pin_1  +3V3
    2    Pin_2  …/IFG_RX               2    Pin_2  GND
    3    Pin_3  …/IFG_INT              3    Pin_3  Net-(J1-Pin_3)
    4    Pin_4  …/IFG_TX               4    Pin_4  Net-(J1-Pin_4)
    5    Pin_5  …/IFG_GND              5    Pin_5  Net-(J1-Pin_5)
    6    Pin_6  SHIELD
```

The add-on's nets lead to its converter: `J1.3` through `R3` to its `RX`,
`J1.4` through `R4` from its `TX`, `J1.5` through `R5` from its `~DRDY`. So
the carrier's `TX` (`J9.4`) goes to `J1.3`, its `RX` (`J9.2`) to `J1.4`, and
its interrupt (`J9.3`) to `J1.5`; the supply and its return go to `J1.1` and
`J1.2`, and `J9.6`, the shield, to nothing:

```toml
[[mate]]
a = "EDGE.J9"
b = "DS2.J1"
map = [["1", "1"], ["5", "2"], ["4", "3"], ["2", "4"], ["3", "5"]]
```

The rest of the file is the bench: 12 V and its return on the carrier's
`J2`, the servo domain's 5 V and return on `J21`, and the force domain's
return and the add-on's analog supply, which nothing on the boards makes.
The module takes its 5 V from the carrier's own regulator, through the
mate.

`embsim check` refuses the project today, naming exactly two parts: the
carrier's RS-422 line driver `U24` and line receiver `U25`, which the
catalog does not model yet (section 9). Every other part of the three
boards is placed, and the mates and wires are checked once the boards
build. `board/tests/edge_project.rs` builds the file with those two parts
given the models the board tests use, and holds the mates to the
hand-written harnesses the machine tests use (every finger and every cable
pin joined as they join it, every empty socket contact and the shield
open); `board/tests/edge_project_live.rs` runs it, and the module's core
rail and the add-on's supply come up from the carrier's rails.

## 7. Adding kinds: a catalog of your own

A crate with boards, models, processor cores or bench parts of its own adds
kinds by implementing `embsim_board::Catalog` for the kinds it has, and adds
that catalog to a **set**, `embsim_boards::catalog::CatalogSet`. A set starts
with the standard catalog, answers each kind from the catalog that provides
it, and is itself a catalog: a project is built with the set. Every method
of `Catalog` but `name` has a default that provides nothing, so a catalog
writes only the methods for the kinds it has:

- **A board kind.** `board_kinds()` names it. `board()` returns a
  `CatalogBoard`: the parsed netlist the crate bundles; the registry it
  builds with, either `Some` registry the catalog builds itself (as
  `Ec32mb::new().registry()` places every part of the module but `U100`) or
  `None`, the set's base registrations, as a `kind = "netlist"` board starts
  from (`CatalogBoard::from_base`); and `models`, the board's own
  `[[board.model]]` entries (`ModelSpec::by_part`, `by_mpn`, `by_value`,
  with `option`). The project registers the board's entries through the
  set's kinds, with every check an entry in the file gets, before the
  project's own entries; a project entry with the same key replaces the
  board's. The project then builds the board with `Board::from_netlist`,
  like every other board.
- **A part kind.** `part_kinds()` returns a `KindGuide` for each
  (`embsim_board::kind`, `KindGuide::new(name, summary, is)`): what the
  model is, the part numbers it is for, its pin tables, the options it
  cannot go without, and `is`, what a part must be for the kind to seat
  there (`Named::Family`, `Connector`, `Switch`, `OneNet`). The project
  checks every part an entry reaches against `is` before it calls the
  catalog (`KindGuide::check`), so no kind, whichever catalog it is from,
  seats on a part it is not (section 8). `register_part()` takes the options
  one by one through `PartOptions` (`choice`, `string`, `number`, `integer`,
  `duration`, `pairs`, and `value` for a shape the kind reads itself) and
  calls `finish()`, which refuses any option the kind did not take and names
  the ones it does. It then registers the model under `assignment.key`, with
  `PartRegistry::register_model` and the `ModelFacade` of the pins the
  component declares, so that the survey checks the pins without building
  anything. `assignment.parts` is every part the key reaches, for a kind
  that reads its configuration from a part's value or number
  (`assignment.nets_of(reference)` gives the nets a part's pins join).
  `assignment.dir` is the project file's directory, for a path an option
  names, `assignment.reports` is where what the model's constructor builds
  reports to a run (section 10), and `assignment.error` puts the entry in
  front of a message.
- **Base registrations.** `register_base()` registers the models a
  `kind = "netlist"` board places by the part number it carries.
- **A bench component kind.** `component_kinds()` names it. `component()`
  takes a `ComponentRequest`: the entry, its `[component.options]` as
  `PartOptions`, the project file's directory and the report sink. It
  returns the `Box<dyn Component>`, whose pins are the endpoints `Name.Pin`.
- **A P2 core.** What runs inside the `p2` package is a core kind, from an
  `embsim_boards::p2::CoreCatalog` added with `CatalogSet::add_cores`
  (section 10).
- **What a kind may not do.** It must not start anything when it registers.
  A survey registers every entry and builds nothing, so a thread starts, or
  a chip boots, in the model's constructor, when the board is built. The
  QEMU core boots QEMU there. The model's numbers stay the model's, with
  their citations: an option chooses among what the model offers.
- **Names.** A kind is lowercase letters, digits and hyphens, and board,
  part, component and core kinds share one namespace. `netlist` is the board
  kind every project has, and no catalog provides it. Two catalogs in a set
  may each provide a name; a project that names it is refused, naming both
  (section 10, "How kinds are named").

The example below, a doc test of `embsim-boards`, adds a board kind that
bundles the header board, and a part kind for the P2-EC32MB's option switch
with the pole pairing the module's netlist gives it
(`embsim_boards::ec32mb::dip_switch_poles`):

```rust
use embsim_board::{
    netlist, Assignment, BoardSpec, Catalog, CatalogBoard, KindGuide, Named, PartOptions,
    PartRegistry, Project, ProjectError,
};
use embsim_boards::catalog::CatalogSet;
use embsim_boards::ec32mb::dip_switch_poles;

/// The board this crate ships: its netlist, bundled.
const HEADER: &str = r#"(export (version "E")
  (components
    (comp (ref "J1") (value "Header")
      (libsource (lib "Connector") (part "Conn_01x02")))
    (comp (ref "R1") (value "10k")
      (libsource (lib "Device") (part "R"))))
  (nets
    (net (code "1") (name "SIG")
      (node (ref "J1") (pin "1") (pinfunction "Pin_1"))
      (node (ref "R1") (pin "1")))
    (net (code "2") (name "GND")
      (node (ref "J1") (pin "2") (pinfunction "Pin_2"))
      (node (ref "R1") (pin "2")))))"#;

/// One board kind and one part kind, beside the standard catalog's.
struct MyCatalog;

impl Catalog for MyCatalog {
    fn name(&self) -> &str {
        "my-catalog"
    }

    fn board_kinds(&self) -> Vec<String> {
        vec!["my-header".to_string()]
    }

    fn board(&self, spec: &BoardSpec) -> Result<CatalogBoard, ProjectError> {
        let netlist = netlist::parse(HEADER)
            .map_err(|err| ProjectError::message(format!("board {}: {err}", spec.name)))?;
        // Nothing on this board needs more than the base registrations.
        Ok(CatalogBoard::from_base(netlist))
    }

    fn part_kinds(&self) -> Vec<KindGuide> {
        // A kind says what the part is: this one seats only on a switch.
        vec![KindGuide::new(
            "my-option-switch",
            "the P2-EC32MB's four-pole option switch",
            Named::Switch,
        )]
    }

    fn register_part(
        &self,
        registry: &mut PartRegistry,
        assignment: &Assignment<'_>,
        options: PartOptions,
    ) -> Result<(), ProjectError> {
        // This kind takes no options; `finish` refuses any it is given.
        options.finish()?;
        // Every part the key reaches is a switch: the project checked. This
        // kind's poles are the CTS part's, so it checks the number too.
        for part in assignment.parts {
            if part.mpn.as_deref() != Some("218-4LPSTJR") {
                return Err(assignment.error(format!(
                    "{} is not the CTS 218-4LPSTJR this kind is for",
                    part.reference
                )));
            }
        }
        // Four poles, each position between its ON and OFF pads.
        registry.register_switch(assignment.key, dip_switch_poles());
        Ok(())
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut set = CatalogSet::new();
    set.add(MyCatalog)?;

    // The board kind: two of the bundled board, wired signal to signal.
    let pair = Project::parse(
        r#"
        [[board]]
        name = "LEFT"
        kind = "my-header"

        [[board]]
        name = "RIGHT"
        kind = "my-header"

        [[wire]]
        from = "LEFT.J1.1"
        to = "RIGHT.J1.1"
        "#,
    )?;
    let built = pair.instantiate(&set)?.build()?;
    assert!(built.names_are_merged("LEFT.SIG", "RIGHT.SIG"));

    // The part kind, on the P2-EC32MB's netlist: the switch is placed, and
    // three parts are left for the project.
    let ec32 = Project::parse(
        r#"
        [[board]]
        name = "EC32"
        kind = "netlist"
        netlist = "netlists/p2_ec32mb.net"

        [[board.model]]
        mpn = "218-4LPSTJR"
        kind = "my-option-switch"
        "#,
    )?;
    let survey = ec32.survey(&set, "EC32")?;
    let left: Vec<&str> = survey.needs_model.iter().map(|part| part.reference.as_str()).collect();
    assert_eq!(left, ["NC_Net", "PCB", "U100"]);
    Ok(())
}
```

A catalog of your own runs from Rust, as above, or in the `embsim` command:
the command is a library, `embsim_cli`, and a binary of ten lines runs it
over a set your catalogs joined (section 10). The `embsim` binary itself is
that command over the catalogs embsim ships, `embsim_cli::shipped()`.

## 8. The rules a project cannot break

- **One pipeline.** Every board is built by `Board::from_netlist` with a
  part registry, the constructor every board in embsim goes through, and it
  is surveyed first with that same registry. A part no model classifies
  stops the board, and the survey is the error (DESIGN.md rule 1). There is
  no stub, no facade-only part and no allow-list. The only way past the
  survey is a class that says what the part is: a model, a switch's poles, a
  connector, or `mechanical` for a part with nothing electrical.
- **A kind is what the part is.** The project checks every part an entry
  reaches against what the kind says it is, before the catalog registers
  anything, for every catalog's kinds (`KindGuide::check`), and refuses a
  part the kind is not, naming what the kind is for and what the board says
  of the part. A model's kind seats on a
  part whose part name, manufacturer part number or value contains the part
  family the model is for. `switch` seats on a part whose designator, symbol
  or name says switch or jumper; `boundary` and `sd-card` on one whose
  designator or symbol says connector; `mechanical` on one whose pins sit on
  one net at most. A pin table that matches says nothing: any part with as
  many pins matches a numbered one.
- **Nothing invented.** The file never holds a model's behaviour or numbers.
  An option chooses among what a model offers, and a model reads what it
  needs from the netlist or its datasheet. The numbers the file does hold,
  a supply's `volts` and the switch and jumper positions, are the bench and
  the scenario (DESIGN.md rule 6: a scenario line). A board has no ground
  and no input supply until a wire gives it one.
- **Wires land on connectors.** A harness joins boards at their boundary.
  A wire's board end is a connector pin, a mate joins two connectors, and
  only a `[[pin_short]]`, a scenario fault, joins any two part pins.
- **One key, one model; one name, one source; one kind, one meaning.** Two
  `[[board.model]]` entries with one key, by any fields, are refused, and so
  are two wires with `volts` from one `from`. A kind two catalogs in the set
  provide is refused where the project names it, naming both (section 10).
- **Everything named is checked before the system starts.** An unknown key,
  kind or option is refused with the ones that exist. So are a key that
  reaches no part, a model that another key or the part's symbol comes
  before, a wire endpoint that is not there, a mate pin with nothing to land
  on, and a switch pole or jumper the part does not have. The error text
  says what to fix.
- **Deterministic.** `run` is stepped: two runs of one project print the
  same report, apart from the line with the wall time.

## 9. Not yet

### Parts the catalog does not model yet

A part on a board embsim ships or tests, that no kind of the standard
catalog is for. A project with one does not build until a kind for it ships
(section 7 for a catalog of your own); the survey names each as needing a
model.

| Part | Board, reference | What exists, and what is owed |
|---|---|---|
| TI AM26LS31 quad RS-422 line driver (`AM26LS31CD`) | MaD Edge, `U24` (the servo step and direction pairs on `J21`) | `Rs422Driver` in `board/tests/machine_parts/mod.rs`, the model the board tests run the Edge board with. It is test-tree code, not a catalog model: its outputs drive through a 25 Ω source impedance no datasheet line gives; their high level is a voltage the test passes in (the servo domain's 5 V), not the part's own supply pin; its pin table declares channels 3 and 4 as passive pins because this board leaves them unwired; and its provenance block asks for per-behaviour datasheet citations (SLLS114N) before it moves out of the tests. |
| TI AM26LV32 quad RS-422 line receiver | MaD Edge, `U25` (the encoder pairs on `J20`) | `Rs422Receiver`, beside the driver, with the same 25 Ω output, the same voltage passed in, and channel 4's inputs declared passive for this board; its input thresholds, input resistance and fail-safe bias are cited (SLLS202H). The netlist disagrees with itself here: the symbol is the 3.3 V `AM26LV32xD`, the manufacturer part number field the 5 V `AM26LS32CD`, and the alternate part number field `AM26LV32IDR`. A model is one part's, so which part the board carries has to be settled first; the survey flags the disagreement. |

`boards/projects/edge-ec32-ds2.toml` waits on these two, and
`board/tests/edge_project.rs` builds and runs it with the test models in a
catalog of the test tree's own beside the standard one
(`machine_parts::edge_catalogs`).

### The rest

- The standard catalog's bench component kinds are `host-serial` and
  `scripted-source` (section 5). A `host-serial` for a host that must run on
  the board's clock (a browser co-simulated in a VM that stops when the
  board's clock does) is not one of them, and neither is a pace for `run`
  against wall time: `run` is stepped, so a quiet system's virtual time runs
  ahead of a host's wall time.
- The standard catalog has no part kind for a diode, LED, FET or transistor.
  One the element library does not know by part number has no way into a
  project yet.
- The scenario lines are switches, jumpers and pin shorts. DNP and value
  overrides, stuck nets and lifted pins are `embsim_board::Scenario` calls
  from Rust. `System::scenario` replaces the scenario the project built, so
  a scenario set from Rust has to carry the project's lines too.
- The `sd-card` kind needs a card image. There is no blank card.
- There is no command that makes a P2 flash image (section 5).
- `run` prints findings in their Rust form (`FloatingSense { … }`). At the
  end it reads again only the findings about a net (a
  floating sense, an unsourced power net, a down rail, a fight, a domain
  with no reference); every other finding is listed as about the board,
  and the engine itself never withdraws a finding.
- The mates of a module and its carrier, or of a cable, are written by
  hand from the two surveys: nothing in the netlists says which connectors
  mate.
- `check` names the parts whose pin table does not fit but not the table
  that would. `survey` and `new` do name it.

## 10. Extending embsim from a project

*Status, 2026-10-01 on `feat/embsim-catalogs`. Landed, and described here as
it is: the catalog set and every extension point (board, part, base, core
and bench component kinds), the rule for a name two catalogs provide, the
check that a kind seats only on a part it is, made by the project for every
catalog, the reports a run prints, the command as a library
(`embsim_cli`), the two bench component kinds of section 5, and `run`'s
interrupt. Still the design, its Rust marked `ignore`: the `[catalog]`
table and the runner that builds a project's own crates into the command,
and a plant's `Assembly`. The decision record, with the alternatives and
what is open, is [`NODES.md`](NODES.md) §13.*

A project whose boards need something embsim does not ship (a model, a
board, a processor core, a bench part) writes it in Rust, in a crate of its
own: a catalog crate. Today the project runs the `embsim` command from a
binary of its own, ten lines over the command's library, with its catalogs
in the set (below). The design that lets the `embsim` command itself build
such a crate in, named by the project file, is "The runner". Either way
there is one command and no plugin interface: Cargo compiles the project's
crate and embsim into one binary, against one copy of embsim.

### What a project can add

Everything a project file names is a **kind**, and a catalog provides each
kind. A project's catalog crate can add five sorts of thing:

| It adds | What it is | Rust | The file names it as |
|---|---|---|---|
| a board kind | a named board: a netlist the crate bundles, and the models its parts take | `Catalog::board_kinds`, `Catalog::board` | `[[board]] kind = "mad-edge"` |
| a part kind | a model, registered into a board's part registry for every part a key reaches | `Catalog::part_kinds`, `Catalog::register_part` | `[[board.model]] kind = "mad-sd-card"` |
| base registrations | models a netlist board places by the part number it carries | `Catalog::register_base` | nothing: every `kind = "netlist"` board starts from them |
| a P2 core | what runs inside the `p2` package | `embsim_boards::p2::CoreCatalog` | `[board.model.options] core = "mad-p2iss"` |
| a bench component | a part with pins and no board: a host port, a stimulus, a plant | `Catalog::component_kinds`, `Catalog::component` | `[[component]] kind = "mad-machine"` |

A mechanical link is not a sixth sort. A plant is one bench component. Its
mechanism (a motor's shaft, a carriage, a sample, a load cell) is inside it,
in Rust. Its outside is electrical pins: a drive's step and direction
inputs, an encoder's outputs, a switch's loop, a bridge's terminals, and
the reference each of them is measured against. The project wires those
pins like any other ([`DESIGN.md`](DESIGN.md) rule 2: one interface, no
second channel).

### The catalog crate

A catalog crate is an ordinary library crate. It depends on the embsim
crates it builds on, and it exports one function at its root, the
**registration function**:

```rust,ignore
// SIL/catalog/src/lib.rs
use embsim_board::ProjectError;
use embsim_boards::catalog::CatalogSet;

/// Called once, after the catalogs embsim ships are in the set and before
/// the project is read.
pub fn register(set: &mut CatalogSet) -> Result<(), ProjectError> {
    // Board, part and component kinds: one `embsim_board::Catalog`.
    set.add(MadCatalog)?;
    // A P2 core: one `embsim_boards::p2::CoreCatalog`.
    set.add_cores(IssCores)?;
    Ok(())
}
```

The registration function:

- is `pub fn register(set: &mut CatalogSet) -> Result<(), ProjectError>`,
  at the crate root, under that name;
- adds catalogs with `CatalogSet::add` and P2 cores with
  `CatalogSet::add_cores`;
- starts nothing: no thread, no file opened, no chip booted. `survey` and
  `new` read the set and build nothing. A thread starts, or a chip boots,
  in a constructor, when a board or the bench is built (section 7, "What a
  kind may not do"), and a constructor with something to say in a run adds
  its report then ("What a run prints", below);
- returns an error that says what to fix.

A project's binary is then the command over the shipped set with the
project's catalogs in it (`embsim_cli`'s crate docs):

```rust,ignore
// SIL/sim/src/main.rs
use std::process::ExitCode;

fn main() -> ExitCode {
    let mut set = embsim_cli::shipped();
    match mad_sim_catalog::register(&mut set) {
        Ok(()) => embsim_cli::main_with(set),
        Err(err) => {
            eprintln!("error: {err}");
            ExitCode::FAILURE
        }
    }
}
```

`embsim_cli::shipped()` is the standard catalog with QEMU's core;
`embsim_cli::main_with(set)` reads the process's arguments and runs the
four subcommands over `set`; `embsim_cli::run(&set, args, out, err)` is the
same with the arguments and the output handed in, which is how
`cli/tests/library.rs` checks and runs a project naming a board kind, a
part kind, a core and a bench component of a catalog the test defines. The
`embsim` binary is `main_with(shipped())`.

### How kinds are named

- **One name, one meaning.** In a set, board, part, component and core
  kinds share one namespace, and a kind means the same thing in every
  project that names it: no catalog replaces another's kind, so section
  5's tables stay true of every project that builds. Two catalogs in a set
  may still each provide a name. The set holds both, and a project that
  **names** that kind is refused, the error naming the kind and both
  catalogs:

  ```text
  error: board EC32: [[board.model]] mpn = "218-4LPSTJR" (kind "twin-switch"): kind "twin-switch" is provided by 2 catalogs, catalog-a and catalog-b; a kind means one thing, so a catalog a project adds gives each of its kinds a name no other catalog has, its project's prefix in front (PROJECTS.md §10)
  ```

  A project that does not name it builds. So when embsim later ships a
  kind a project's catalog also has, only the projects that name that kind
  are asked to choose, and a project's catalog that puts its project's name
  in front of every kind it adds (`mad-machine`, `mad-p2iss`) is never
  asked.
- **Base registrations clash the same way.** A part number two catalogs'
  base registrations place parts by refuses a board that carries such a
  part, naming the part, the number and both catalogs, unless an entry
  gives that key a kind itself.
- **What a set refuses at once.** A catalog joining a set is refused,
  before any project is read, for what is wrong whatever a project names: a
  kind not spelled as a kind is (lowercase letters, digits and hyphens);
  `netlist`, the board kind every project has; one name as two sorts of
  kind of one catalog; and a catalog under a name the set already holds (a
  core catalog may share its crate's catalog's name).
- **Choosing a name.** Put your project's name in front of every kind your
  catalog adds (`mad-machine`, `mad-edge`, `mad-p2iss`). Part-family names
  (`am26ls31`) are the ones the standard catalog uses, and a model a
  project writes for a generic part is one embsim should ship
  (section 9).

An unknown kind is refused as before, listing every kind the set holds.
`check` and `run` print the set's catalogs under the project line:

```text
project mad.toml
  catalogs: embsim-boards, embsim-p2-qemu, mad-sim-catalog
```

### The `[catalog]` table

*Design; not built.*

| Key | What it says |
|---|---|
| `crates` | the project's catalog crates: each a directory holding a `Cargo.toml`, relative to the project file. Their registration functions run in this order, after the catalogs embsim ships |
| `embsim` | optional: the embsim checkout (its workspace root, relative to the project file) that the runner builds embsim from. By default it is the checkout `embsim` itself was built from |

A key the table does not have is refused, as everywhere in the file. A
project without `[catalog]` runs on the catalogs embsim ships, in the
`embsim` binary itself, exactly as sections 1 to 9 describe.

### The runner

*Design; not built. The library it would call is built: `main_with(set)`
runs a project in the process that calls it and never hands one over, so a
binary of the project's own (above) is already the runner a project writes
by hand. `NODES.md` §13, "Open", records the review of this design and the
smaller one it proposed: a runner crate the project owns, named by the
file, which `embsim` builds and hands over to.*

A project with `[catalog]` runs in a **runner**: a small crate that `embsim`
writes, builds with Cargo, and hands the command line to.

- **What it is.** A binary crate whose dependencies are `embsim-cli`, from
  the embsim checkout the runner builds against, and each crate `[catalog]`
  names, by path. Its `main` is the binary above, one registration function
  per crate.
- **Where it lives.** In `.embsim/runner-<id>/` beside the project file:
  `Cargo.toml`, `main.rs` and a `build.rs` that links QEMU the way the
  `embsim` binary's own does. `<id>` is a hash of the crates' canonical
  directories and the embsim checkout, so two projects that name the same
  crates share a runner. The manifest declares an empty `[workspace]`, so
  the runner is never taken for a member of a workspace it sits inside.
  Keep `.embsim/` out of version control.
- **Who runs a project.** The `embsim` binary hands a project with
  `[catalog]` to its runner; a runner, like every binary built on
  `main_with`, runs the project itself. Before it hands over, `embsim` reads
  only the `[catalog]` table, so a project that the runner's embsim reads is
  never refused by an older `embsim`.
- **When it builds.** On every subcommand, `embsim` rewrites the runner's
  three files only if their content would change, then runs `cargo build`
  on the runner. The first build compiles embsim and the catalog crates.
  After that Cargo rebuilds only what changed, and with nothing changed the
  build is Cargo's own no-op check. Cargo's progress goes to standard error,
  with one line before it:

  ```text
  embsim: building the runner for mad.toml (mad-sim-catalog, embsim at /home/me/MaD/SIL/embsim) in .embsim/runner-3f9a2c1e
  ```

  The profile is `release`, unless `EMBSIM_RUNNER_PROFILE` names another.
  The target directory is `CARGO_TARGET_DIR` when it is set. Otherwise it
  is the target directory of the workspace the first catalog crate belongs
  to (MaD's `SIL/target`), so the crates that workspace has already built
  are reused, and failing that `.embsim/target`. The runner's `Cargo.lock`
  starts as a copy of that workspace's lock file (else embsim's), so shared
  dependencies keep the versions they were tested at. Cargo resolves only
  what neither file names.
- **How it hands over.** `embsim` takes the runner's path from Cargo's build
  report and `exec`s it with the same arguments. The runner is then the
  process: its output, its exit status and Ctrl-C are its own. embsim is a
  Unix program (its PTYs are), so there is no other path.
- **When the build fails.** The command exits 1 with Cargo's errors above
  one line saying which runner did not build, from which crates, against
  which embsim.

#### Which embsim the runner builds against

*Design; not built, and under review (`NODES.md` §13, "Open").*

The runner's `embsim-cli` and every catalog crate's embsim dependencies must
be one copy of embsim. `embsim` picks the checkout in this order:

1. `[catalog] embsim`, when the project gives it;
2. the checkout `embsim` was built from. Its `build.rs` records the
   workspace root at build time (`EMBSIM_SOURCE_DIR`).

QEMU is linked into a runner when it was linked into `embsim`.
`EMBSIM_QEMU_P2_BUILD` is passed through when it is set in the environment,
and otherwise the tree `embsim` was built with is used (recorded the same
way). Without either, the runner refuses `core = "qemu"` with the message
section 5 shows.

### Adding a P2 core

The `p2` part kind is the package: its 86 pins, the START gate (the reset,
`VDD` and the datasheet's 3 ms restart delay), the bank supplies, the
brownout hold (`embsim_boards::p2`). What runs inside it is a **core
kind**. The package takes `core` from the entry's options, asks the set's
core catalogs for that kind (`CoreCatalog::seat`), and hands the core the
rest of the options, so each core takes its own: `rom` is the `qemu`
core's, and `held-in-reset` takes none. The seat returns the constructor
the board build calls once per part; the `p2` kind wraps whatever it
returns in `P2Package::new` (`embsim_boards::p2::register_p2`), so no core
can skip the START gate. A core implements `P2Core` (`attach(P2Pads)`,
`start`, `reset`) and drives its pads through
`P2Pads::bank_supplies().pad_drive(..)`, as QEMU's does. A constructor that
cannot make its core returns the reason, and the package refuses to attach
with it, so the system does not start.

The example below, a doc test of `embsim-boards`, adds a core that holds
the chip in reset and records nothing, and seats it in the P2-EC32MB:

```rust
use embsim_board::{Assignment, PartOptions, Project, ProjectError};
use embsim_boards::catalog::CatalogSet;
use embsim_boards::p2::{CoreCatalog, CoreCtor, CoreKind, HeldInReset, P2Core};

/// One core kind, `my-core`.
struct MyCores;

impl CoreCatalog for MyCores {
    fn name(&self) -> &str {
        "my-catalog"
    }

    fn core_kinds(&self) -> Vec<CoreKind> {
        vec![CoreKind { name: "my-core", summary: "a core that runs nothing" }]
    }

    fn seat(
        &self,
        _core: &str,
        _assignment: &Assignment<'_>,
        options: PartOptions,
    ) -> Result<CoreCtor, ProjectError> {
        // Every option but `core` is this core's; it takes none.
        options.finish()?;
        // Called at board build, once per part; a survey never calls it.
        Ok(Box::new(|_decl| Ok(Box::new(HeldInReset) as Box<dyn P2Core>)))
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut set = CatalogSet::new();
    set.add_cores(MyCores)?;
    let project = Project::parse(
        r#"
        [[board]]
        name = "EC32"
        kind = "p2-ec32mb"

        [[board.model]]
        value = "P2X8C4M64P"
        kind = "p2"
        [board.model.options]
        core = "my-core"
        "#,
    )?;
    assert!(project.survey(&set, "EC32")?.compliant());
    Ok(())
}
```

The standard catalog's core is `held-in-reset`. QEMU's is `qemu`, added by
`embsim_p2_qemu::catalog::register` like any project's, and it refuses a key
that reaches two parts, since QEMU is one machine per process. The `p2`
kind adds a report for every package it seats (its start, or why it is
held), and a core adds its own (QEMU's console, per pad).

### Adding a board

`Catalog::board` returns a `CatalogBoard`: the parsed netlist the crate
bundles (`include_str!` of an EDA export), the registry it builds with, and
`models`, the board's own `[[board.model]]` entries. The registry is either
a registry the catalog builds itself (`p2-ec32mb`'s, which places every part
of the module but the processor) or `None`, meaning start from the set's
base registrations, as a `kind = "netlist"` board does
(`CatalogBoard::from_base`). The project registers the board's `models`
through the set's kinds before the project's own entries, with every check
an entry in the file gets. An entry in the project with the same key
replaces the board's, as a project's flash image replaces the P2-EC32MB's
blank flash.

```rust,ignore
fn board(&self, spec: &BoardSpec) -> Result<CatalogBoard, ProjectError> {
    let netlist = netlist::parse(MAD_EDGE_NET)
        .map_err(|err| ProjectError::message(format!("board {}: {err}", spec.name)))?;
    // J3, the module socket, is a project-library symbol: a connector.
    Ok(CatalogBoard::from_base(netlist)
        .with_model(ModelSpec::by_part("P2_EDGE_MODULE_SOCKET", "boundary")))
}
```

### Adding a part model

`Catalog::part_kinds` returns a `KindGuide` for each kind
(`embsim_board::kind`). This is the same guide the standard catalog's kinds
have, and `survey` and `new` read it to name a kind for a part. Its `is`
says what a part must be for the kind to seat there (`Named::Family`,
`Connector`, `Switch`, `OneNet`). The project checks `is` for every part an
entry reaches before it calls the catalog's `register_part`, whichever
catalog the kind comes from (`KindGuide::check`), and refuses a part the
kind is not, as section 8 says: a project's kinds are held to rule 1 by the
pipeline, not by each author remembering the check. `register_part` then
does what section 7 describes: it takes its options through `PartOptions`
and registers the model under `assignment.key` with its `ModelFacade`. The
model's numbers carry their citations ([`DESIGN.md`](DESIGN.md) rules 6 and
9 bind a project's models as they bind embsim's).

### Adding a bench component

```toml
[[component]]
name = "MACHINE"
kind = "mad-machine"
[component.options]
sample = "sil-linear-reference"
loop_volts = 24.0
```

`Catalog::component` receives a `ComponentRequest`: the entry
(`spec`), its `[component.options]` as `PartOptions` (`options`), the
project file's directory (`dir`), and the report sink (`reports`). It
returns the `Box<dyn Component>`, whose pins are its endpoints, `Name.Pin`.
`PartOptions` takes `number`, `integer`, `duration` (a time, written as
`--for` takes it: `"1.5ms"`) and `value` (a shape the kind reads itself)
beside `string`, `choice` and `pairs`. The two kinds of section 5 are built
this way (`boards/src/catalog.rs`).

*Design; not built:* a component made of models embsim already has would
build them into an **`embsim_board::Assembly`**: one component that hosts
several, each part's pins renamed onto the assembly's, every part reaching
the engine through the one interface, its pins through the assembly's
handle table and its wakes through a `WakeGate` the assembly holds, as the
P2 package hosts its core. At a shared instant the assembly would wake its
parts in the order they were added. The links between the parts (a shaft
turning an encoder, a carriage opening a switch) are Rust inside the
assembly; the engine sees one node with pins, and no node sees another
(rule 4). `NODES.md` §13 records what such a plant samples, and how often.

### What a run prints about what a catalog built

A core's console, a PTY's path and a carriage's travel are not findings,
and no net carries them. Anything a catalog builds can hand the run a
**report** (`embsim_board::Report`):

```rust,ignore
// embsim_board::report
pub trait Report: Send {
    /// What the lines are about, as the run prints it: "EC32.U100", "HOST".
    fn subject(&self) -> String;
    /// What is new since the last look, at `now_ns` of the run's virtual time.
    fn look(&mut self, now_ns: u64) -> Vec<String>;
    /// The state at the end of the run.
    fn summary(&self) -> Vec<String>;
}
```

A constructor adds its report to the sink the build handed it
(`assignment.reports` for a part's, `request.reports` for a bench
component's; `Reports::add`). The run builds the system with
`Project::instantiate_with(&set, &reports)`, takes the reports once it is
built (`Reports::take`), asks each at every look and at the end, and prints
what they return under their subject, stamped like a finding. A look reads
state the engine's thread wrote while the run's thread was parked, so on
the stepped clock two runs print the same report, as section 3 says.

### MaD, end to end

*Design. The kinds MaD's catalog adds are not written, the Edge carrier's
RS-422 pair has no kind yet (section 9), and the file below has not been
built: its harness was checked against the Edge netlist
(`board/tests/fixtures/mad_edge.net`) by reading it, not by a test.*

The MaD tensile tester (`RileyMcCarthy/MaD`, `SIL/`) is the consumer this
design is checked against. Today its SIL is `mad-emulator`
(`SIL/MaDSim/src/main.rs`), a program that assembles the system by hand:

- the P2 instruction-set simulator as a component on no board, declaring
  only the pins its lists name;
- the DS2 add-on board;
- a PTY;
- bench pull-ups;
- a stepper, an encoder and two end switches, coupled by callbacks;
- the gantry, sample and strain-gauge chain;
- an SD card node.

As a project it is one crate, one binary and one file. `NODES.md` §13 maps
each of `mad-emulator`'s pieces to a kind and lists what MaD has to change
first.

**The crate**, `SIL/catalog` (`mad-sim-catalog`), adds:

| Kind | Sort | What it is |
|---|---|---|
| `mad-p2iss` | P2 core | `p2iss::P2Iss` as a `P2Core`, booting the chip's ROM off the board's flash as `qemu` does. Option `rom` (a file; by default the boot ROM `p2iss/rom/rom_booter_v33k.bin`) |
| `mad-edge` | board | the MaD Edge carrier, from a netlist exported from `Hardware/EdgeBoard`; its socket `J3` a `boundary` |
| `mad-sd-card` | part | a FAT16 card holding a directory (`p2iss::sdimage::mad_card`), embsim's SD card model in a connector. Option `dir` (required) |
| `mad-machine` | component | the machine: servo drive, carriage, encoder, end switches, door and emergency stops, gantry, sample and load cell, one plant |

`mad-machine`'s pins and its options:

| Pins | What they are |
|---|---|
| `STEP`, `DIR`, `ENA` | the servo drive's inputs (`embsim_models::machine::StepperMotor`, `DIR` low forward, enable active low), read against `DRIVE_GND` |
| `DRIVE_GND` | the drive's input return, on the carrier's `EN_GND` (`J21.8`) |
| `ENC_A`, `ENC_B`, `ENC_Z` | the encoder's outputs (`QuadratureEncoder`, as many counts per millimetre as steps), against `ENC_GND` |
| `ENC_GND` | the encoder's return, on `EN_GND` (`J20.5`) |
| `UPPER+`, `UPPER-`, `LOWER+`, `LOWER-`, `DOOR+`, `DOOR-`, `ESD_U+`, `ESD_U-`, `ESD_L+`, `ESD_L-`, `ESD_A+`, `ESD_A-` | each switch as the sourced loop the carrier reads: `+` the machine's loop supply behind its normally-closed contact while the contact is closed, released while it is open; `-` that supply's return. The carrier's loops are a current regulator and an opto LED between `+` and `-` (`IC9` and `U6` for the upper end switch), and power nothing themselves |
| `E+`, `E-` | the load cell's excitation, sensed: the bridge reads its excitation off the board instead of assuming 3.3 V |
| `S+`, `S-` | the bridge's outputs, each behind the cell's 350 Ω, at the excitation's midpoint ± half the strain gauge's output |
| `sample` (option) | the sample in the grips, one of the samples the crate carries with their provenance: `sil-linear-reference` |
| `loop_volts` (option) | required: the machine's switch-loop supply. A scenario line names it (rule 6); the isolation test's bench uses 24 V |

The rest of its numbers (8192 steps a millimetre, 100 mm of travel, the
gantry's 15 mm of slack, the cell's sensitivity) are the machine's. They
stay in the crate with their citations, as they are in `MaDSim` and `models`
today.

**The file**, `SIL/mad.toml`:

```toml
# The MaD tensile tester: the Edge carrier, the P2-EC32MB module in its
# socket running the firmware on MaD's instruction-set simulator, the DS2
# force-gauge add-on on the force cable, the machine on the carrier's
# connectors, and the Raspberry Pi's serial port as a PTY.

[catalog]
crates = ["catalog"]     # SIL/catalog, the crate mad-sim-catalog

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

# Stage-1 and the propeller2_debug program, laid out by `make flash-image`
# (p2iss::flashimage): the ROM boots the firmware off the module's flash.
[[board.model]]
value = "SPI Flash 16MB (128Mb)"
kind = "w25q128jv"
[board.model.options]
pins = "by-function"
image = "build/flash.bin"

[[board.model]]
value = "MicroSD Socket"
kind = "mad-sd-card"
[board.model.options]
dir = "sd"

[[board]]
name = "DS2"
kind = "netlist"
netlist = "boards/ds2_addon.net"

[[board.model]]
part = "ADS122U04"
kind = "ads122u04"

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

# ---- The module in its socket, and the force cable (edge-ec32-ds2.toml) ----

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
from = "MACHINE.ENC_A"
to = "EDGE.J20.1"

[[wire]]                 # B+
from = "MACHINE.ENC_B"
to = "EDGE.J20.3"

[[wire]]                 # ZI+, U25's third channel (J20.7 and .8 are its enables)
from = "MACHINE.ENC_Z"
to = "EDGE.J20.9"

[[wire]]                 # EN_GND, the encoder's return
from = "MACHINE.ENC_GND"
to = "EDGE.J20.5"

# The switch loops follow the nets: IEND_U is on J16, IEND_L on J15 and
# IDOOR on J14, whatever the silkscreen says (board/tests/machine_parts,
# machine_harness). Pin 2 of each is the loop's + (the current regulator's
# anode), pin 1 its - (the opto LED's cathode).
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
part = "EDGE.JP1"        # through R24 (pads A and C)
pole = 0
state = "closed"

[[jumper]]               # A-, B- and ZI- to the encoder ground: a
part = "EDGE.JP2"        # single-ended encoder on the RS-422 receiver
state = "closed"

[[jumper]]
part = "EDGE.JP3"
state = "closed"

[[jumper]]               # Z_GND: Z-, U25's active-low enable, to EN_GND
part = "EDGE.JP4"
state = "closed"

[[jumper]]               # ZI_GND
part = "EDGE.JP5"
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

Which of `JP1`'s two positions the machine's drive takes (pads A and C, the
TTL output; pads B and C, `Q1`'s open collector) is the machine's, and so is
`loop_volts`; the file states the first and the isolation test's 24 V, and
both are for MaD to confirm (`NODES.md` §13, "Open").

**The commands** that replace `mad-emulator` (`SIL/makefile`), today through
MaD's own binary (`cargo run -p mad-sim --`) and, once the runner is built,
through `embsim` itself:

```bash
cd SIL
embsim check mad.toml                        # every board, wire and kind checked; time held
embsim run mad.toml --pty /tmp/tty.rpi       # make playground / e2e-emulator: until Ctrl-C
embsim run mad.toml --for 2s --net EDGE.+3.3V
```

Until embsim ships kinds for the Edge board's RS-422 pair (`U24`, `U25`;
section 9), `check` refuses `EDGE` and names those two parts, as it refuses
`edge-ec32-ds2.toml` today. `make playground-rom`'s serial boot is a second
file: the same boards with `S301` set for serial boot, an erased flash, and
the host on the carrier's `Debug` header (`J1`: `P62`, `P63`, `RESn`).
