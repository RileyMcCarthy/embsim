# Board engine design (`embsim-board`)

**Status:** design accepted, not yet implemented (2026-07-11, revised after adversarial review)

`embsim-board` turns embsim from a firmware-centric emulator ("firmware in the
middle, models hand-wired around it") into a **component-centric system
simulator**: boards are ingested from EDA netlists, every component — including
the MCU — is a `Component` with named pins, and an engine resolves the nets
between them. Consumers stop writing wiring code and start writing *system
descriptions*.

This document specifies the generic engine. Consumer-side specifics (part
registry entries, harness files, plant models) live in the consuming repo — see
MaD's `docs/dev/sil-board-simulation-design.md` for the reference consumer and
the decision record for why this is netlist-structural rather than SPICE.

## Crate layout

```
board/                    # new workspace member: embsim-board
├── src/netlist.rs        # KiCad s-expression netlist parser → ComponentDecl/NetDecl graph
├── src/component.rs      # Component trait, PinDecl, PinKind, StreamRole, ComponentNetIo
├── src/registry.rs       # PartRegistry: identity → constructor; auto-classification tiers
├── src/engine.rs         # net-engine thread: drive queue, resolution, timer wheel, diagnostics
├── src/net.rs            # net state model, Thevenin drive resolution, digital projection
├── src/cluster.rs        # analog cluster extraction + quasi-static MNA solver (trait)
├── src/stream.rs         # stream endpoints (serial byte pipes) derived from net routes
├── src/board.rs          # Board::from_netlist(netlist, registry) → components + nets
├── src/system.rs         # System: boards + harnesses + scenario overrides + fault algebra
└── tests/                # parser fixtures (per KiCad version), net truth tables, MNA hand-checks
```

## Execution model (single-writer net engine)

All net state is owned by **one net-engine thread**. Everything else —
firmware cores, model threads, sense callbacks — interacts with it through two
lock-free paths:

- **Drives are enqueued**, never applied inline: a pin drive is an MPSC message
  `(endpoint_id, new_drive, enqueue_seq)`. The engine thread dequeues,
  serializes, assigns the authoritative event order, resolves affected nets,
  and updates `NetState`.
- **Senses are delivered** from the engine thread with **no engine lock held**.
  The re-entrancy contract: a sense callback MAY drive a pin; that drive is
  enqueued and resolved in a later engine iteration — it is never resolved
  inline. This makes feedback loops (driver → net → sense → drive) well-defined
  and deadlock-free by construction.
- **Time-driven behavior** is engine-owned: components do not get a broadcast
  `tick()`. They request wakeups via their I/O handle
  (`io.schedule_at(v_us)` / `io.schedule_every(v_us)`), served by a timer wheel
  on the engine thread keyed to virtual time. Idle components cost nothing.

Note on time: `embsim_core::virtual_clock` is **free-running scaled wall time**
(no step/pause API, no central tick loop), so wakeup timestamps are sampled,
not deterministic. Time-sensitive state must be computed at *read time* (as the
RC closed form is), never integrated per tick.

## Core abstractions

### `Component`

```rust
pub trait Component: Send + Sync {
    /// Declared pins. Must cover the component's netlist pins exactly —
    /// build validates BOTH directions (declared-but-absent and
    /// present-but-undeclared netlist pins are hard errors).
    fn pins(&self) -> &[PinDecl];
    /// Runs once at build, BEFORE the component is shared (pre-Arc), so
    /// components store typed pin handles without interior mutability and
    /// fail loudly on facade mismatch.
    fn attach(&mut self, io: ComponentNetIo) -> Result<(), AttachError>;
}

pub struct PinDecl {
    pub number: &'static str,        // netlist pin number ("3")
    pub name: Option<&'static str>,  // alias ("RX") — matches KiCad pinfunction when present
    pub kind: PinKind,
    pub stream: Option<StreamRole>,  // serial endpoints; see "Stream endpoints"
    pub drive_impedance: Option<Ohms>, // Thevenin source impedance; default per kind
}

pub enum PinKind {
    DigitalIn,     // senses net level; contributes no drive
    DigitalOut,    // push-pull Thevenin driver (default 25 Ω)
    DigitalBidir,  // driver with runtime direction (GPIO)
    Analog,        // participates in cluster solve (high-Z sense, source, or
                   //   parameterized primitive — see "Transducer components")
    PowerIn,       // consumes a power domain
    PowerOut,      // sources a power domain at a declared voltage
    Passive,       // terminal of a passive primitive (R/C/L/jumper)
}
```

Pin identity matching against the netlist: **pinfunction if present, else pin
number**, with KiCad overline syntax normalized (`~{RESET}` ≡ `~RESET`).

Concurrency contract for component internals: sense callbacks and scheduled
wakeups are all delivered from the engine thread, so they never race each
other; they MAY race the component's own protocol threads (e.g. a serial
handler), which remains the component's responsibility, as today.

### Net state model — one mechanism, digital as a projection

Every driver is a **Thevenin source** (voltage + impedance; push-pull digital
defaults to 25 Ω, overridable per `PinDecl`). Nets connected through passives
form clusters (below); resolution always happens at cluster granularity, and
the familiar digital states are a **derived view** of the solved node voltage —
not a parallel mechanism (so "a pull-up is just a resistor" causes no
ambiguity):

```rust
pub enum NetState {
    Floating,          // no source reaches this node (MNA singular for the node)
    Driven(Level),     // solved V within V_OL/V_OH of a rail, dominated by one push-pull source
    Pulled(Level, Ohms), // rail-adjacent V dominated by a resistive path
    Analog(Volts),     // none of the above projections apply — raw node voltage
    Contention,        // ≥2 push-pull sources fighting (directly or through
                       //   collapsed low-value series resistance)
}
```

Resolution and escalation rules:

- Purely digital nets (single push-pull driver, no passives) short-circuit the
  solver: `Driven(level)`.
- If a competing path's Thevenin impedance is within 10× of the strongest
  driver's, the net escalates to the cluster solver and resolves to the actual
  divided voltage. Digital senses then apply their declared `V_IH`/`V_IL`
  thresholds; a solved voltage inside the dead band raises an
  **`AmbiguousLevel`** finding. This is how a real mid-rail fight (25 Ω driver
  low vs 3.3 V through 47 Ω) is representable *numerically*, not just as a flag.
- Two disagreeing push-pull sources on one node — or coupled through series
  resistance below the stream-collapse threshold — resolve to `Contention` plus
  a structured diagnostic (net, drivers). A finding, never a panic.
- `Floating` senses are reported to the sensing component, which chooses
  datasheet behavior (silent chip for a floating `~RESET`, noise policy for a
  floating ADC input). The engine never invents a value silently.
- A **full resolution pass runs at `System::build()`**, so never-driven nets
  (the one-pin `~RESET` net) are reported `Floating` to their sensing
  components immediately, before any traffic.

### Power domains

Power is **volts, not booleans**:

```rust
pub struct PowerState { pub volts: f64, pub ok: bool }
```

- `PowerOut` pins source their net at a declared voltage; those rails enter
  cluster solves as real sources (the MNA needs values, and chip models need
  the numeric AVDD for range checks like PGA common-mode).
- A power net with **no `PowerOut` source anywhere** (board or harness) raises
  a **`PowerNetUnsourced`** finding and presents as down — this is precisely
  the "AVDD unstrapped" failure mode.
- A down domain presents its rail nodes to cluster solves as 0 V sources (not
  removed), so dependent analog senses read the physical consequence.
- **No implicit net-name merging across boards.** Two boards both naming a net
  `GND` share nothing until a harness connects them — grounds included. An
  unreferenced ground is a finding, not an assumption.
- Harness endpoints may declare a power kind
  (`from = "P2EVAL.3V3", kind = "power(3.3V)"`) so bench rigs can source
  domains without a designed PCB.

### Analog clusters

Connected subgraphs of `Passive`/`Analog` pins form **clusters**, extracted at
build time. Solved by quasi-static modified nodal analysis (MNA): Thevenin
sources + resistors → node voltages, recomputed only when a boundary input
changes. Single-pole RC behavior is closed-form (time constant annotated on the
cluster; senses read the exponential at read time — no fixed-timestep
integration).

- A cluster with **no source** solves to `Floating` for all its nodes (the MNA
  detects singularity — it never returns garbage), reported to senses like the
  digital floating clause.
- **Transducer components** may contribute *parameterized primitives* to a
  cluster — e.g. a load-cell component contributes four bridge-leg resistors
  whose values are driven by the consumer's physics plant. Common-mode and
  differential voltages then fall out of the same MNA as everything else,
  rather than being hand-computed inside a model.

```rust
pub trait ClusterSolver: Send + Sync {
    fn solve(&self, cluster: &Cluster, inputs: &ClusterInputs) -> ClusterSolution;
}
```

The trait is the deliberate seam: the default is `QuasiStaticMna`; a transient
SPICE-backed solver is a possible future implementation and is intentionally
**not** part of this design (no ngspice dependency, no cluster-marking syntax —
see the consumer decision record for the rationale and revisit trigger).

### Stream endpoints (serial over pins)

A UART link is two nets carrying byte streams. Byte pipes are **derived from
and gated by net resolution**, never installed beside it:

- A serial-capable pin declares `stream: Some(Producer { baud })` or
  `Consumer { baud }`; its `PinKind` stays digital, with a declared idle drive
  (UART TX idles `Driven(High)`).
- At build (and on any topology-affecting change: jumper toggles, faults,
  harness swaps), the engine routes each `Producer` to `Consumer`s reachable
  through its net **and through series passives below a collapse threshold
  (default < 1 kΩ)** — so the DS2Addon's 47 Ω series resistors correctly
  collapse into the link.
- Bytes flow on the derived route with baud pacing. Route validity is
  re-checked when the underlying nets change; a broken route stops delivery.
- Compatibility findings at routing time: a route with two `Producer`s facing
  each other (the crossed-TX/RX harness) raises **`StreamMismatch`**, and the
  underlying nets — one with opposing push-pull sources through collapsed
  resistance, one with no producer — additionally resolve per the net rules
  (`Contention` / idle-high with a silent consumer). The regression test
  asserts the findings, and the mid-rail voltage is available from the
  escalated solve for scenarios that want it.
- Byte-loss fidelity is explicitly **not** modeled at pipe granularity (no
  RX-overrun emergence); scenario-driven byte-drop fault injection on a stream
  is the supported way to exercise loss-handling code.

### Netlist ingestion

Input: KiCad s-expression netlist (`kicad-cli sch export netlist`). Parsing is
**version-gated** on `(export (version …))` — unsupported versions fail with a
named error, and the test suite carries one fixture per supported KiCad major.

Parsed per component: `ref`, `value`, `footprint`, `libsource (lib, part)`,
sheetpath (hierarchical designs), fields/properties; per net: `code`, `name`,
`(ref, pin, pinfunction?)` nodes.

Classification is three-tier and **keyed primarily on the libsource *part*
name** — the lib name is best-effort only (real exports contain empty lib names
and KiCad `*-rescue` libs; rescue mangling like
`DS2_Addon-rescue::Jumper_NO_Small-Device` is normalized before matching):

| Tier | Match | Result |
|---|---|---|
| auto | part `R*`/`C*`/`L*`/`LED`/`D_*` from `Device` (or rescue thereof) | passive primitive; value parsed (`47R`, `4k7`, `0.1uF`); **pin-count validated** — a 2-terminal class with ≠2 pins is a hard classification error (resistor arrays etc. need a registry expansion entry) |
| auto | `Conn*`/`Screw_Terminal*` parts | board boundary pins (harness attachment points) |
| auto | `Jumper*` parts | stateful short; default from name (`_NO`/`_Open` → open, `_NC`/`_Bridged` → closed; 3-pin `Jumper_3_*` variants get a selectable position) |
| auto | `MountingHole*`/`Logo*`/`TestPoint*` | ignored |
| registry | anything else, keyed by part name, falling back to `value` | consumer-registered `Component` constructor; the registry API includes an **expansion hook** (one netlist component → N primitives, for arrays/multi-unit symbols) |
| error | no registry match | system construction fails; per-board explicit stub list is the only escape |

Active parts come from standard *and* custom libraries alike (`74xGxx`,
`Isolator`, `Interface`, `Transistor_BJT`, `Switch` are all standard-lib
actives on real boards) — tier 2 is "whatever tier 1 does not match," not
"custom-library symbols."

DNP: a component with `value == "X"` (consumer convention) or the KiCad `dnp`
property is absent from the built board. Jumper state and DNP overrides are
**scenario** inputs.

### Boards, harnesses, systems, scenarios

```rust
let ds2   = Board::from_netlist(parse(ds2_net)?, &registry)?;
let edge  = Board::from_netlist(parse(edge_net)?, &registry)?;
let sys   = System::new()
    .board("EdgeBoard", edge)
    .board("DS2Addon", ds2)
    .harness(Harness::from_toml(bench_toml)?)   // connector-pin ↔ connector-pin (+ power endpoints)
    .scenario(Scenario::default()
        .jumper("DS2Addon.JP1", Closed)
        .pin_detach("DS2Addon.U1.3"))           // fault algebra, see below
    .build()?;
```

The **fault algebra** is defined in terms of graph primitives the netlist
actually has (an N-pin net has no "where" for a generic open):

- `pin_detach("Board.Ref.Pin")` — remove one node from its net (a lifted pin /
  cold joint; the floating-`~RESET` case);
- `pin_short(a, b)` — union two nets (solder bridge, crossed probe);
- `net_stuck(net, rail)` — add a Thevenin source to a net;
- `value_override("Board.R5", "4k7")`, `dnp_override("Board.C7", Populated)` —
  scenario-time BOM changes;
- `stream_drop(endpoint, policy)` — byte-loss injection on a serial route.

Harness endpoints are `Board.Connector.Pin` references; bare MCU-pin endpoints
(`P2EVAL.P0`) are allowed for bench rigs that aren't a designed PCB.
Deliberately wrong harnesses (swapped pins) are valid fixtures — the
`StreamMismatch`/`Contention`/`Floating` findings are the assertion targets.

## The MCU as a component

A platform crate (per `CONTRACT.md`) provides the MCU component:

1. **Firmware image**: the consumer's static library. The engine **spawns** the
   entry on a component-owned thread (today `embsim_runtime::Emulator::run`
   executes the entry synchronously on the caller's thread and blocks;
   `System::build` must return, so this moves). CONTRACT.md's init-ordering
   section is re-stated for the board engine: `attach()` and net service must
   be live before firmware entry.
2. **Peripherals**: the generic peripheral emulations become fields of the MCU
   component instance rather than process globals. The full global-state
   inventory this de-globalizes: serial (`CHANNEL_FDS`/baud/pacing), GPIO
   (state + callbacks), pulse-out, encoder, i2c, plus the MCU-internal ones
   that gain no pins but still must be per-instance — locks, the thread
   registry, the filesystem mount, and per-MCU clock frequency. The
   `#[no_mangle]` trampolines then need **thread-identity routing** to the
   owning instance (registered at `startThread` time). This is a CONTRACT.md
   revision — the current contract explicitly assumes no indirection — and is
   why de-globalization is its own phase. (A given firmware image's own C
   statics still limit that image to one instance per process.)
3. **Pin facade**: the HAL-channel → physical-pin map is read from the
   firmware's HAL config tables. **Prerequisite (consumer-side):** those tables
   must exist in the natively-linked binary — extracted into *data-only*
   translation units (no HAL function definitions, no MCU intrinsics) with
   **external linkage**, unique `HAL_`-prefixed names, and
   `__attribute__((used))`, compiled into the consumer's native library. The
   read path is `embsim-memory-inspect`'s **SymbolResolver + DWARF layout**
   (reading initialized data values by symbol — a different path from the
   DWARF-type-only enum lookup consumers use today, same crate). A CI check
   asserts the tables are present and non-empty before the emulator boots, so
   "table optimized away" is a build failure, not a mystery unwired pin.

Channel behavior stays HAL-granular (byte pipes, GPIO levels); pins are
topology. Baud and channel parameters come from the same tables — the emulator
stops inventing its own defaults (consumers may keep explicit pacing overrides
for tests).

Behavioral fidelity boundary, stated explicitly: **no cycle-accurate silicon
emulation.** Raising fidelity of one peripheral later (bit-timed serial, PWM
edges) is an internal change behind the same pin interface.

## Model provenance convention

Every behavioral model is only as trustworthy as its sourcing, so provenance is
a requirement, not a nicety:

- **Datasheet-backed parts** (real purchasable silicon): the module doc comment
  names the datasheet document number and revision (e.g. `TI SBAS752B, Oct
  2018`) and which sections govern which parts of the module. Every implemented
  behavior carries a short in-place citation with **section and printed page**
  (e.g. `// RREG = 0010 rrrx, replies 1 byte (SBAS752B §8.5.3.5, p.37)`).
  Deliberate simplifications are annotated as such, citing what the full
  behavior would be. Behavior with no datasheet basis is a defect.
- **Physics/mechanical models** (plants, transducers): the header states the
  governing equation or derivation and the parameter source (product listing,
  machine spec). Magic numbers without provenance are flagged in review.

## Diagnostics

Structured findings on a diagnostics bus, mirrored to `tracing`, consumable by
tests (assert a specific finding fired) and by trace tooling later:
`Contention`, `FloatingSense` (digital and analog), `AmbiguousLevel`,
`PowerNetUnsourced`, `StreamMismatch`, `ClassificationError`,
`UnconnectedRegistryPin` (both directions: declared-but-absent and
present-but-undeclared).

## Testing conventions

- Parser: committed netlist fixtures per supported KiCad major (hand-written
  minimal + one real exported board) with golden component/net graphs.
- Net resolution: truth-table tests per rule (driver combinations × expected
  `NetState`), including the impedance-escalation boundary.
- MNA: hand-computed reference circuits (bridge, divider ladder, pull-up vs
  driver, source-free singular cluster) asserted to µV.
- Streams: routing through series passives, crossed-producer detection, route
  invalidation on jumper/fault changes.
- System: a two-component smoke board (fake MCU pin driver + fake sensor)
  exercising attach/schedule/diagnostics without any consumer firmware.

## Non-goals

- SPICE/transient analog simulation (trait seam reserved; not built).
- Cycle-accurate MCU peripheral timing; emergent byte-loss on serial pipes
  (fault injection covers loss-handling code paths).
- PCB physical effects (parasitics, thermal, EMC).
- Auto-generating *plant* physics — transducer components expose parameterized
  primitives (e.g. bridge legs) for consumer physics models to drive.
