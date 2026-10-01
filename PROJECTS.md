# embsim projects — a system in a file

embsim runs one `System`: boards, bench components, the harness wires between
them, and a scenario. A **project** is that system written down, in a TOML
file, and the `embsim` command takes a netlist to a running system through
one: `survey` says what the netlist asks for, `new` writes a starter project,
`check` builds it, `run` runs it. Rust code loads the same file with
`embsim_board::Project`. This document is the guide to both: what a project
is, the workflow, the kinds the standard catalog ships, wiring boards to each
other, adding kinds of your own, and the rules a project cannot break
([`DESIGN.md`](DESIGN.md) holds the rules for the whole of embsim).

## 1. What a project is

A project holds four lists, and each one is part of the `System` it builds:

| In the file | In the `System` | What it is |
|---|---|---|
| `[[board]]`, each with its `[[board.model]]` entries | `System::board` | a board: a netlist, either a KiCad export or one a catalog board kind bundles, built with a part registry, and the model each part the registry cannot place takes |
| `[[component]]` | `System::component` | a bench component: a part with pins and no board |
| `[[wire]]` | `System::harness` | the harness: each wire joins two endpoints, or, with `volts`, sources one |
| `[[switch]]`, `[[jumper]]`, `[[pin_short]]` | `System::scenario` | the scenario: switch poles and jumpers opened or closed, and two part pins shorted |

The file names **kinds**. A catalog (`embsim_board::Catalog`) turns each
kind into what it is: a board's netlist and part registry, a part model
registered into that registry, or a bench component. The standard catalog is
`embsim_boards::catalog::StandardCatalog`, and the `embsim` command uses
`embsim_p2_qemu::catalog::QemuCatalog`, which is the same catalog with QEMU
as a core the P2 can hold (section 5). The file itself holds no behaviour.
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
| `[[component]]` | `name`, `kind` | a bench component and its catalog kind; endpoints on it are `Name.Pin` |
| `[[wire]]` | `from`, `to` | two endpoints; a board's is `Board.Connector.Pin` |
| | `volts` | optional: `from` becomes a source at this voltage (section 6) |
| `[[switch]]` | `part`, `pole`, `state` | a switch pole: `part = "EC32.S301"`, the pole numbered from 0 in the order the part declares its poles, `state = "open"` or `"closed"` |
| `[[jumper]]` | `part`, `state` | a two-pad jumper (a `Jumper*` or `SolderJumper*` symbol), open or closed; a three-pad one is two switch poles (pads 1–2 and 2–3) |
| `[[pin_short]]` | `a`, `b` | two part pins, `Board.Ref.Pin`, joined as one net: a scenario fault or a bodge wire |

A board or component name is not empty and has no dot or space, and no two
are the same. A path in the file (`netlist`, and the `image` and `rom`
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
  number), its pins, and the kinds that could be it. A kind fits by part
  number first: compared on their letters and digits, one of the part's keys
  and a number the kind is for are the same, or one starts with the other, as
  `ADS122U04` starts `ADS122U04IPW`. A key with fewer than five letters and
  digits names no part. Failing that, a kind fits by a pin table with exactly
  the part's pins, and failing that, by a table with as many pins. The survey
  shows only the strongest kind of fit any kind makes.
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
  by its manufacturer part number, else its part name, else its value. When
  exactly one kind fits by part number or by pins, the stub names that kind.
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
give each part that needs a model a [[board.model]] with its part, mpn or value and a kind; the part kinds are "p2", "tg2520smn", …
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

Choose a kind by what the part is, and a table by the netlist's pins. When no
kind fits, the part needs a model that nobody has written yet (section 7).
Three kinds say what a part is without a model. `switch` pairs the part's
pins into poles. `boundary` makes it a connector. `mechanical` says the part
has nothing electrical: a PCB line, a layout node, a mounting hole. The build
reports a mechanical pad on a net a pin drives (`MechanicalOnDrivenNet`).

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
  1 board, 0 bench components, 4 wires
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
  …
running for 5.000000 ms of virtual time
[   0.000000 ms] FloatingSense { net: "DS2.GPIO1", kind: Digital }
…
ran 5.000000 ms of virtual time in 0.001 s
net DS2.+3V3: Analog(3.3)
net DS2.VDDA: Analog(3.3)
net DS2.~RESET: Floating
findings: 11
```

`run` starts the system on a stepped virtual clock and runs it for `--for`
of virtual time. The duration is a number and a unit: `ns`, `us` (or `µs`),
`ms` or `s`. While it runs, `run` looks at the system every 100 µs of
virtual time. It prints each new finding stamped with the instant of the
look that found it, and, for a P2 running on QEMU (section 5), the instant
its core started and its console output. At the end it prints the nets named
with `--net` (`Board.Net`, as the board's netlist spells the net). A net the
system does not have is refused before the run starts.

Virtual time advances only to the next instant something happens, so two
runs of a project print the same report, apart from the line with the wall
time. Without `--for`, a run lasts until it is interrupted (Ctrl-C) and
prints nothing more.

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
| kind | model | placed by part number | `pins` (the first is the default) | other options |
|---|---|---|---|---|
| `p2` | the Propeller 2 package | never (it is for `P2X8C4M64P`) | fixed: `P2X8C4M64P` (86 pins) | `core` = `"held-in-reset"` (required) |
| `tg2520smn` | EPSON TCXO; frequency from the part's value or number | `TG2520SMN 20.0000M-ECGNNM3` | `"numbered"` (4 pins), `"by-function"` (4 pins) | — |
| `74lvc2g04` | NXP dual inverter | `74LVC2G04GW,125` | `"sot363"` (6 pins), `"by-function"` (6 pins) | — |
| `sn74lvc1g14` | TI Schmitt inverter | `SN74LVC1G14DBVR`, `SN74LVC1G14DBVT` | `"sot23"` (5 pins) | — |
| `aps6404l` | AP Memory PSRAM | `APS6404L-3SQR-ZR` | `"sop8"` (8 pins), `"by-function"` (9 pins) | — |
| `w25q128jv` | Winbond serial NOR flash, blank or holding an image | `W25Q128JVSIM`, `W25Q128JVSIM TR`, `W25Q128JVSIQ` | `"soic8"` (8 pins), `"by-function"` (8 pins), `"spi-only"` (4 pins) | `id` = `"im"`, `"iq"`; `image = "boot.bin"` — a file the part holds from address 0, the rest erased, relative to the project file |
| `sd-card` | a card in an SD socket | — | `"microsd"` (8 pins), `"by-function"` (8 pins), `"spi-only"` (4 pins) | `image = "card.img"` — the card in the socket: a card image file, relative to the project file (required) |
| `ap62301` | Diodes buck; setpoint from its feedback divider | `AP62301Z6-7` | `"sot563"` (6 pins), `"by-function"` (5 pins) | — |
| `ncp114` | onsemi LDO; setpoint from the part's value or number | `NCP114AMX330TCG` | `"udfn4"` (4 pins), `"by-function"` (5 pins) | — |
| `xl1509` | XLSEMI buck; version from the part's value or number | `XL1509-3.3E1`, `XL1509-5.0E1`, `XL1509-12E1` | `"sop8"` (8 pins) | — |
| `ucc12040` | TI isolated DC/DC; setpoint from its SEL strap | `UCC12040DVE`, `UCC12040DVER` | `"soic16"` (16 pins) | — |
| `stm1061` | ST voltage detector, from its ordering code | `STM1061N16WX6F` | `"sot23"` (3 pins), `"by-function"` (3 pins) | — |
| `6n137` | Lite-On optocoupler | `6N137` | fixed: `6N137` (7 pins) | — |
| `vo2631` | Vishay dual optocoupler | `VO2631` | fixed: `VO2631` (8 pins) | — |
| `iso67xx` | TI digital isolator, the member the key names | `ISO6721BDR`, `ISO6731DWR`, `ISO6740DWR`, `ISO6740FDWR`, `ISO6741DWR`, `ISO6742DWR` | fixed, the member's: `ISO6721BDR` (8 pins), `ISO6731DWR` (16 pins), `ISO6740DWR` (16 pins), `ISO6740FDWR` (16 pins), `ISO6741DWR` (16 pins), `ISO6742DWR` (16 pins) | — |
| `ads122u04` | TI 24-bit ADC, as it comes out of reset | `ADS122U04IPW`, `ADS122U04IPWR` | `"tssop16"` (16 pins) | — |
| `switch` | a switch whose poles pair the part's pins, each open | — | the part's own | `poles = [["1", "2"]]` — the part's pins paired into poles, each open until a [[switch]] closes it (required) |
| `mechanical` | a part with pads and nothing electrical | — | the part's own | — |
| `boundary` | a connector, by its symbol's part name | — | the part's own | — |
<!-- part-kinds:end -->

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

The standard catalog seats one core in the `p2` package: `"held-in-reset"`,
the chip before it runs, with its pads released. The `embsim` command's
catalog, `embsim_p2_qemu::catalog::QemuCatalog`, also takes
`core = "qemu"`, which boots the P2 on QEMU off whatever the board gives
it. The boot ROM is Parallax's own (`embsim_p2_qemu::BOOT_ROM`) unless the
`rom` option names a file. On the P2-EC32MB the chip boots from its flash,
which the `w25q128jv` kind's `image` option fills:

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
[  10.000000 ms] EC32.U100 P62: "B"
ran 20.000000 ms of virtual time in 0.044 s
EC32.U100 (core "qemu"): started at 5.500000 ms; 16953 pad yields; running; console P62 "B"
net EC32.Common_VDD: Analog(1.8133333333333335)
```

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

## 6. Board to board

A harness attaches to a board at its boundary. A wire's end on a board is a
connector pin, `Board.Connector.Pin`. Its other end is one of these:

- a connector pin on the same board or another one;
- a bench component's pin, `Name.Pin`;
- a supply. A wire with `volts` makes its `from` a source at that voltage.
  That end is usually a name no board or component has (`BENCH.5V`), and any
  other wire may join it.

A wire to a pin that is not on a connector is refused, and the error lists
the connectors:

```text
error: [[wire]] BENCH.GND to HDR.R1.2: R1 is not a connector, and a wire lands on a connector pin; HDR's connectors are J1
```

A `[[pin_short]]` is the one way to join two pins that are not on
connectors. It is a scenario line, for a fault or a bodge wire, and it may
join any two part pins on the boards.

[`boards/projects/header-pair.toml`](boards/projects/header-pair.toml) wires
two boards together. Each board is the header board (`header.net`), a
two-pin connector `J1` with a 10 kΩ resistor from its `SIG` to its `GND`.
The file joins the two `J1`s pin for pin and puts the bench's 0 V on the
left one's ground:

```toml
[[board]]
name = "LEFT"
kind = "netlist"
netlist = "header.net"

[[board]]
name = "RIGHT"
kind = "netlist"
netlist = "header.net"

[[wire]]
from = "LEFT.J1.1"
to = "RIGHT.J1.1"

[[wire]]
from = "LEFT.J1.2"
to = "RIGHT.J1.2"

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
  2 boards, 0 bench components, 3 wires
build findings: none
ok: boards/projects/header-pair.toml builds
```

Each board keeps its own net names. The wire makes `LEFT.SIG` and
`RIGHT.SIG` one node, and `BuiltSystem::names_are_merged` says so (the
first example in section 1).
[`boards/tests/board_to_board.rs`](boards/tests/board_to_board.rs) proves
it live. It drives a 25 Ω pad on `LEFT.J1.1`, and `RIGHT.SIG` reads
3.3 V divided between the pad and both boards' resistors in parallel.

## 7. Adding kinds: a catalog of your own

A crate with boards or models of its own adds kinds by implementing
`embsim_board::Catalog`, and passes everything else to the standard catalog.
`QemuCatalog` (`p2-qemu/src/catalog.rs`) is one: it adds a core to the `p2`
kind and passes every other kind, option and board to `StandardCatalog`.

- **A board kind.** Add its name to `board_kinds()`. `board()` returns a
  `CatalogBoard`: the parsed netlist the crate bundles, and the registry the
  board builds with. That registry places every part of the board but the
  ones a project chooses, as `Ec32mb::new().registry()` leaves `U100` for
  the project. The project then builds it with `Board::from_netlist`, like
  every other board.
- **A part kind.** Add its name to `part_kinds()`. `register_part()` takes
  the options one by one through `PartOptions` (`choice`, `string`, `pairs`)
  and calls `finish()`, which refuses any option the kind did not take and
  names the ones it does. It then registers the model under
  `assignment.key`, with `PartRegistry::register_model` and the
  `ModelFacade` of the pins the component declares, so that the survey
  checks the pins without building anything. `assignment.parts` is every
  part the key reaches, for a kind that reads its configuration from a
  part's value or number. `assignment.dir` is the project file's directory,
  for a path an option names, and `assignment.error` puts the entry in
  front of a message.
- **What a kind may not do.** It must not start anything when it registers.
  A survey registers every entry and builds nothing, so a thread starts, or
  a chip boots, in the model's constructor, when the board is built.
  `QemuCatalog` boots QEMU there. The model's numbers stay the model's,
  with their citations: an option chooses among what the model offers.

The example below, a doc test of `embsim-boards`, adds a board kind that
bundles the header board, and a part kind for the P2-EC32MB's option switch
with the pole pairing the module's netlist gives it
(`embsim_boards::ec32mb::dip_switch_poles`):

```rust
use embsim_board::{
    netlist, Assignment, BoardSpec, Catalog, CatalogBoard, Component, ComponentSpec, PartOptions,
    PartRegistry, Project, ProjectError,
};
use embsim_boards::catalog::StandardCatalog;
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

/// The standard catalog, with one board kind and one part kind more.
struct MyCatalog;

impl Catalog for MyCatalog {
    fn board_kinds(&self) -> Vec<String> {
        let mut kinds = StandardCatalog.board_kinds();
        kinds.push("header".to_string());
        kinds
    }

    fn board(&self, spec: &BoardSpec) -> Result<CatalogBoard, ProjectError> {
        if spec.kind != "header" {
            return StandardCatalog.board(spec);
        }
        let netlist = netlist::parse(HEADER)
            .map_err(|err| ProjectError::message(format!("board {}: {err}", spec.name)))?;
        // Nothing on this board needs more than the base registry.
        Ok(CatalogBoard { netlist, registry: StandardCatalog::base_registry() })
    }

    fn base_registry(&self) -> PartRegistry {
        StandardCatalog::base_registry()
    }

    fn part_kinds(&self) -> Vec<String> {
        let mut kinds = StandardCatalog.part_kinds();
        kinds.push("ec32-option-switch".to_string());
        kinds
    }

    fn register_part(
        &self,
        registry: &mut PartRegistry,
        assignment: &Assignment<'_>,
        options: PartOptions,
    ) -> Result<(), ProjectError> {
        if assignment.kind != "ec32-option-switch" {
            return StandardCatalog.register_part(registry, assignment, options);
        }
        // This kind takes no options; `finish` refuses any it is given.
        options.finish()?;
        // Four poles, each position between its ON and OFF pads.
        registry.register_switch(assignment.key, dip_switch_poles());
        Ok(())
    }

    fn component_kinds(&self) -> Vec<String> {
        StandardCatalog.component_kinds()
    }

    fn component(&self, spec: &ComponentSpec) -> Result<Box<dyn Component>, ProjectError> {
        StandardCatalog.component(spec)
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // The board kind: two of the bundled board, wired signal to signal.
    let pair = Project::parse(
        r#"
        [[board]]
        name = "LEFT"
        kind = "header"

        [[board]]
        name = "RIGHT"
        kind = "header"

        [[wire]]
        from = "LEFT.J1.1"
        to = "RIGHT.J1.1"
        "#,
    )?;
    let built = pair.instantiate(&MyCatalog)?.build()?;
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
        kind = "ec32-option-switch"
        "#,
    )?;
    let survey = ec32.survey(&MyCatalog, "EC32")?;
    let left: Vec<&str> = survey.needs_model.iter().map(|part| part.reference.as_str()).collect();
    assert_eq!(left, ["NC_Net", "PCB", "U100"]);
    Ok(())
}
```

A catalog of your own runs from Rust. The `embsim` binary's catalog is
`QemuCatalog`, fixed when the binary is built, and the binary loads no other.

## 8. The rules a project cannot break

- **One pipeline.** Every board is built by `Board::from_netlist` with a
  part registry, the constructor every board in embsim goes through, and it
  is surveyed first with that same registry. A part no model classifies
  stops the board, and the survey is the error (DESIGN.md rule 1). There is
  no stub, no facade-only part and no allow-list. The only way past the
  survey is a class that says what the part is: a model, a switch's poles, a
  connector, or `mechanical` for a part with nothing electrical.
- **Nothing invented.** The file never holds a model's behaviour or numbers.
  An option chooses among what a model offers, and a model reads what it
  needs from the netlist or its datasheet. The numbers the file does hold,
  a supply's `volts` and the switch and jumper positions, are the bench and
  the scenario (DESIGN.md rule 6: a scenario line). A board has no ground
  and no input supply until a wire gives it one.
- **Wires land on connectors.** A harness joins boards at their boundary.
  A wire's board end is a connector pin, and only a `[[pin_short]]`, a
  scenario fault, joins any two part pins.
- **Everything named is checked before the system starts.** An unknown key,
  kind or option is refused with the ones that exist. So are a key that
  reaches no part, a model that another key or the part's symbol comes
  before, a wire endpoint that is not there, and a switch pole or jumper the
  part does not have. The error text says what to fix.
- **Deterministic.** `run` is stepped: two runs of one project print the
  same report, apart from the line with the wall time.

## 9. Not yet

- The standard catalog has no bench component kinds. `[[component]]` parses,
  and every kind it names is refused.
- The standard catalog has no part kind for a diode, LED, FET or transistor.
  One the element library does not know by part number has no way into a
  project yet.
- The scenario lines are switches, jumpers and pin shorts. DNP and value
  overrides, stuck nets and lifted pins are `embsim_board::Scenario` calls
  from Rust. `System::scenario` replaces the scenario the project built, so
  a scenario set from Rust has to carry the project's lines too.
- A project has no host serial port. `embsim_board::HostPty`, a PTY whose
  bytes are levels on two pins, is a bench component added from Rust with
  `System::component`.
- The `sd-card` kind needs a card image. There is no blank card.
- There is no command that makes a P2 flash image (section 5).
- `run` prints findings in their Rust form (`FloatingSense { … }`).
  Interrupted, it prints no summary.
- `check` names the parts whose pin table does not fit but not the table
  that would. `survey` and `new` do name it.
