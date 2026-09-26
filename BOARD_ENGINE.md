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
├── src/component.rs      # Component trait, PinDecl, PinRole, Drive, ComponentNetIo
├── src/registry.rs       # PartRegistry: identity → constructor; auto-classification tiers
├── src/engine.rs         # net-engine thread: drive queue, resolution, timer wheel, diagnostics
├── src/net.rs            # net state model, Thevenin drive resolution, digital projection
├── src/cluster.rs        # analog cluster extraction + quasi-static MNA solver (trait)
├── src/uart.rs           # UART framing codec (byte ↔ timed digital levels)
├── src/serial_levels.rs  # SerialLevelBridge: that codec on a pin (no byte route)
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
  (`io.schedule_at(v_us)` / `io.schedule_every(v_us)`, or their `_ns` forms),
  served by a timer wheel on the engine thread keyed to virtual time. Idle
  components cost nothing.

Note on time: `embsim_core::virtual_clock` is one nanosecond **counter**
(nanoseconds because a microsecond cannot hold a bit: 500 ns at 2 Mbaud).
The engine is the time authority (`advance_to`). `--speed` only paces the
host after a jump. Time-sensitive state must be computed at *read time*,
never integrated per tick.

### Determinism of this execution model

The ordering rules above (enqueue-seq for drives, `(deadline, schedule)` for the
wheel, per-producer FIFO for streams) make the engine's event order
**consistent** — every observer agrees on one order. Wakeup timestamps are
integers the engine chose (`advance_to`), so a system whose actors are all
engine-hosted produces a byte-identical event trace across runs and processes.
Two actors released at the same instant still race `next_drive_seq`. What
"quiescent" means with pump threads and real fds in the picture, what extending
that across the firmware↔engine boundary would cost, and how CI proves it are in
[`DETERMINISM.md`](DETERMINISM.md).

Practical consequence for component authors: **take your time from the engine.**
A component that gets its cadence from `io.schedule_at` / `io.schedule_every`
and does its work in `on_wake` is deterministic for free,
because it is not a separate actor at all — its callback runs on the engine
thread. A component that spawns a thread with its own poll loop has to register
that thread with `virtual_clock::register_actor` and park through the clock, and
even then it can only be as deterministic as whatever it is polling.
`embsim_models::ads122u04_component` is the reference conversion in both
directions: its output pump became a wheel entry, its model's protocol thread
became a registered actor.

Two engine rules from that document are **in force today** (Phase D0), and both
are enforced by review rather than by the compiler:

- **No `HashMap`/`HashSet` iteration on an engine path without an explicit
  sort.** Walk a dense `Vec` and use the map for keyed lookups, or collect and
  `sort_unstable()`. A set used purely as a membership/dedup gate is fine and
  carries an inline `// hash-order: …` note saying why order cannot escape. The
  rule exists because `std`'s hasher is randomly seeded *per map construction*,
  so an unordered walk that reaches a float accumulation makes the engine
  irreproducible in a way that looks like a last-bit rounding difference. See
  `engine.rs`'s module docs for the sanctioned shapes.
- **The engine's event order is observable**: `System::event_log()` turns on an
  append-only transcript of drives, resolutions, sense deliveries, wakeups,
  stream bytes, reroutes, and findings, with a normalization contract for
  comparing runs (`board/src/event_log.rs`). Off by default.

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
    pub number: &'static str,          // netlist pin number ("3")
    pub name: Option<&'static str>,    // alias ("RX") — matches KiCad pinfunction when present
    pub role: PinRole,                 // Signal | PowerIn | PowerOut | Passive
    pub idle: Option<TheveninDrive>,   // the drive from attach until the component drives
                                       //   (None = released); refused at build on a power-in
                                       //   or passive pin (no slot): BoardError::IdleOnSlotlessPin
    pub input: Option<InputPort>,      // { v_bias, r_in }: the pin's own load, stamped as a
                                       //   permanent weak source at the pin
    pub clamps: &'static [Clamp],      // shunts to the supply/reference pin, stamped as
                                       //   diode branches
    pub capacitance_pf: Option<f64>,   // to the reference (declared; armed in phase 6)
    pub thresholds: Option<Thresholds>,// { v_il, v_ih, hysteresis }: fractions of the supply
                                       //   when the pin names one, else volts
    pub reference: Option<&'static str>, // the pin its voltages are measured against
    pub supply: Option<&'static str>,  // the pin its relative thresholds scale with
    pub can_source: bool,              // drives high
    pub can_sink: bool,                // drives low; neither = an input
}

pub enum PinRole { Signal, PowerIn, PowerOut, Passive }
```

There are no pin kinds (`NODES.md` §11 and §12 item 5): what a
pin reads and drives follows from its role and its declarations. A pin is
built from one of eight constructors and the `with_*` builders — a
constructor that needs thresholds takes them, and there are no crate
default thresholds (a part cites its datasheet's; `jesd8c01_lvcmos_thresholds`,
the 3.3 V LVCMOS pair, is a named constant a caller passes with its reason):

| Declaration | Role | Declared | What the engine does with it |
|---|---|---|---|
| `digital_in(n, thresholds)` | `Signal` | thresholds; neither sources nor sinks | a **digital sense** from the build (a floating one is `FloatingSense`), and a released slot a sense callback may drive through |
| `analog(n)` | `Signal` | no thresholds; neither sources nor sinks | an **analog reader**: its cluster is solved whenever it resolves; a drive through it contradicts its declaration and is traced |
| `analog_source(n)` | `Signal` | no thresholds; sources and sinks; rests released | a **linear source** — a bench supply's output, a bridge's excitation, a pull a bench part stands in for (`with_idle` declares a static one): a drive slot, no sense |
| `digital_out(n)` | `Signal` | idles high at 3.3 V behind 25 Ω; sources and sinks | a drive slot holding its idle until the part drives (`with_impedance`, `with_idle` override) |
| `digital_out(n).sink_only()` | `Signal` | sinks, cannot source; rests released | an open drain: a sink that releases — a build finding, `OpenDrainWithoutPullUp`, when no pull-up reaches its net |
| `digital_io(n, t)` | `Signal` | sources and sinks, thresholds, rests released | a **bidirectional pad**: its net is read once the part subscribes to it |
| `power_in(n)` | `PowerIn` | optionally a reference | sensed as a supply; no slot |
| `power_out(n)` | `PowerOut` | idles at an unmodelled voltage unless `with_idle` names one (`None`: released, what every regulator model declares) | a **declared terminal**: its own one-node cluster, holding what its slot publishes |
| `passive(n)` | `Passive` | — | nothing |

The build refuses a declaration the engine could not keep: an idle drive on
a power-in or passive pin (`IdleOnSlotlessPin`), a reference or supply
naming a pin the part does not declare (`PinFacadeMismatch`, as a branch
naming one is), thresholds that are not fractions on a pin naming a supply
or not volts on one naming none, a clamp to a rail the pin does not declare
or with a negative knee or resistance, an input port on a pin that is
neither a signal nor a power input or without a finite bias and a finite
positive resistance (`InvalidDeclaration`).

Two declarations are stamped into the solve at build and never
republished (`NODES.md` §10). An **input port** (`with_input`) is a
permanent Thevenin source at the pin, `v_bias` behind `r_in`, ranked by
rule 2 like any source — at a kilohm or more a pull, so an otherwise open
input sits at its bias and any real driver wins; the AM26LV32's open-input
fail-safe is its 12 kΩ to its 0.83 V / 0.70 V open-circuit voltages. The
bias is stamped in the engine's frame, exact while the part's ground sits at
0 V. A **clamp** (`with_clamps`) is a diode branch — pin to supply, or
reference to pin — registered as the part's own branches are: an element
ending on its rail's terminal, no union, the region loop deciding it, its
current summed into the pin's. A clamp makes its cluster an element cluster,
solved whenever it resolves; no part on the three boards declares one. `PinHandle::thresholds` reports a pin's thresholds
in volts at the instant — a relative declaration scaled by the voltage its
supply reads against its reference.

### What a node sees

A node never reads the engine's projection. A sensing pin is handed a
`Sense` (`NODES.md` §10 "Delivered to a sensing pin", §11; the sense task of
§12 item 5):

```rust
pub struct Sense {
    pub volts: Option<Volts>,            // against the pin's reference
    pub periodic: Option<PeriodicSense>, // { hi, lo: Option<Volts>, segment }
    pub at_ns: u64,                      // the instant it was delivered
}
```

- **`volts`** is the node's voltage **against the pin's declared
  `reference`** — the engine's frame when it declares none. `None` when no
  source reaches the node (a fact a part branches on: an open enable), and
  when no voltage can be named: a node only an unmodelled rail reaches, a
  clock (see `periodic`), a node with two operating points (a clock fought for
  half of every cycle, two rates meeting), and a pin whose reference itself
  names no voltage — floating, or detached. Ground is not implicit: a bench
  holds the ground its parts measure against, or they are handed nothing. A
  **fought** node with one operating point is handed that voltage — the one
  the `AmbiguousLevel` finding beside the `Contention` names.
- **`periodic`** carries a square wave's two phase voltages (same frame)
  and its `PeriodicSchedule` (integer nanoseconds); the consumer integrates
  the schedule once its two phases cross the consumer's own thresholds
  (`PeriodicSense::rate`); a self-biased input is the exception (`NODES.md`
  §12 item 5, the cleanup and the final pass).
- The engine keeps a voltage beside every state it resolves (`Net::volts`,
  crate-private): a projected node's winning source's open-circuit voltage,
  a solved node's operating point. It is published with the states under the
  same pass, and the build path and the live path hand the same `Sense` for
  the same resolution — one conversion, `PinHandle::measure`.
- A sense is delivered once at registration, then whenever the net's state
  changes, whenever **only the voltage behind it** changes (a `Driven(High)`
  whose source moved from 3.3 V to 1.8 V: `SenseDelivered` is logged, no
  `NetResolved`), and whenever the pin's **reference** moves while its own
  net did not, or its declared **supply** moves (a relative threshold scales
  with it: the same `Sense` again, which a `DigitalReceiver` re-projects
  through the thresholds the moved supply now gives). A subscription whose
  reference and supply both moved is delivered once.

The level is the **receiver's** projection: `Sense::level(&thresholds,
last)` — at or below `V_IL` low and at or above `V_IH` high whatever it read
last; between them the hysteresis chosen by the last level (a high input
stays high down to `V_IH − ΔV_T`, a low one low up to `V_IL + ΔV_T`); what is
left is the dead band, answered by the receiver's declared `DeadBand` —
`HoldLast` (a Schmitt input, a comparator enable) or `Unknown` (a plain CMOS
input, where the datasheet guarantees neither level). The policy is a field
of `Thresholds`, required by its constructor: there is no crate default.
`PinHandle::level` projects through the pin's declared thresholds at the
instant; `DigitalReceiver` keeps the last level for a model. A pin with no
thresholds is handed volts only. `PinHandle::sense` reads the same `Sense`
on demand.

The engine's own projection — `NetState`, `level_of`, the JESD8C.01 pair —
stays the engine's **report**: `BuiltSystem::net_state`, the event log, the
goldens, the census, and an instrument's subscription
(`ComponentNetIo::on_net_report`, `PinHandle::net_report`) that a bench probe
records. No part model reads it.

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

Resolution is **source-strength projection, in one form** (`NODES.md`
"Three rules the taxonomy rests on", rule 2; `engine.rs` `project_root`).
Every Thevenin source reaching a node — a pad, a rail, a `net_stuck` — is
ranked by its **total ohms**: its own impedance plus the minimum series
resistance from its net to the node (a rail or fault is ideal, so the path
alone). Then, per node:

- A source at or above `WEAK_DRIVE_OHMS` (1 kΩ, one value with the stream
  collapse threshold) in total is a **pull**. It sets the level only when
  nothing stronger reaches the node, and it never contends: a 10.5 kΩ pull-up
  against a 25 Ω pad is the pad's node, a 15 kΩ pad against a 30 Ω sink is
  `Driven(Low)` with nothing reported.
- Among the rest, a source `ESCALATION_IMPEDANCE_RATIO` (10×) the strongest's
  total ohms or more **loses**: the node takes the strongest's state, and
  because it lost through less than a kilohm it is a fight —
  `Finding::Contention` names every strong pin on the node. A pad driving
  low against a rail on its own net reads `Analog(rail)` *and* reports the
  fight; the card behind the module's 240 Ω series resistor loses to the
  flash on P58 (265 Ω against 25) and P58 reads the flash's level.
- Disagreeing sources within 10× of each other **solve**: the cluster goes
  through the `ClusterSolver` once for the pass and the node takes the
  divided voltage, projected through the `V_IL`/`V_IH` dead band (0.8/2.0 V,
  JESD8C.01): strictly inside it the node is `Contention` and an
  **`AmbiguousLevel`** finding carries the voltage; outside it the node is
  `Analog(v)` — either way `Contention` names the pins. Two 25 Ω push-pulls on
  one net sit at 1.65 V, inside the band; a 25 Ω pad against a 100 Ω one
  reads `Analog(2.64)` with the finding. Pulls alone disagreeing (a divider
  between two rails) solve to `Analog(v)` with nothing reported.
- Otherwise the winner sets the state: a strong pad on the node is
  `Driven(level)`; a rail or fault on the node is `Analog(v)`; anything else
  is `Pulled(level, ohms)` where `ohms` is the **winner's series path** (plus
  the winner's own impedance when that is itself a kilohm or more — a 15 kΩ
  pad is a resistor to its rail; a 25 Ω pad is a driver). A net one 10.5 kΩ
  resistor from ground reports 10 500, whatever else shares its cluster.
- A node an **analog sense** reads, one a **current injection**
  (`Drive::Current`) lands in, or one a **current instrument** reads is
  published at its cluster's operating point — the sense wants a voltage,
  an injection has no projection form (its effect is `I · R` along whatever
  the node is tied to), an instrument a current — and the fight findings
  above are reported beside it exactly as without the reader: a fought node
  under an ADC reads `Analog(v)` *and* reports `Contention` (with
  `AmbiguousLevel` inside the band), since the reader is handed the
  voltage by its `Sense` (`NODES.md` §12 item 5, the rules task, which
  retired phase 1's operating-point precedence). An injection or an
  instrument solves the whole cluster. An analog sense solves only where it
  must: a root **exactly one source reaches** is handed that source's
  open-circuit voltage without a solve (`DESIGN.md` rule 8) — no other
  source, terminal or injection shares its conduction component, so no
  current flows and the voltage is exact; a root two or more sources reach
  is solved. A current injected where no Thevenin source reaches leaves the
  node `Floating` with `Finding::CurrentIntoFloatingNode`.
- A **declared terminal** — a `PowerOut` pin's net, a harness `power(V)`
  endpoint, a `net_stuck` — is a cluster of its own and a boundary of every
  cluster around it (`NODES.md` "Three rules the taxonomy rests on", 1;
  phase 4): a resistor or an element ends on it and nothing unions through
  it; what it holds is decided once from its sources (`decide_terminal`) and
  enters every dependent's solve as a Dirichlet constant and every
  dependent's ranking as an ideal 0 Ω source through the path to it; two
  sources that disagree on it are one fight, solved and reported once at
  the terminal; a change to what it holds re-resolves the terminal's cluster
  and its fan-out (`Topology::terminals`, `mark_terminal_dirty`). Membership
  is fixed at build — the terminal is declared whatever it holds.
- A rail whose voltage no model declares yet (`PowerOut` at NaN, the facade's
  default idle) sources its cluster as *up*: a node nothing numeric reaches
  reads `Pulled(High, path)` through the path to the nearest such rail —
  the engine's report; a sensing pin there is handed no voltage, since the
  rail names none (`DESIGN.md` rule 6), and a supply gate reads it down. A
  rail its part released holds nothing: its node floats, and its dependents
  read only what else reaches them.
- A `TheveninDrive` behind a non-finite impedance is normalised to
  *released* at the drive slot (`Resolver::set_drive`), so it is never
  ranked and never escalates a cluster.
- A sense on a `Floating` net is handed no voltage, and the sensing
  component chooses
  datasheet behavior (silent chip for a floating `~RESET`, noise policy for a
  floating ADC input). The engine never invents a value silently.
- A **full resolution pass runs at `System::build()`**, so never-driven nets
  (the one-pin `~RESET` net) are reported `Floating` to their sensing
  components immediately, before any traffic. The build then runs to a
  **bounded fixed point**: the drives components issue in response to the
  states they are delivered are replayed and the changed states delivered
  again, until nothing moves (bound `BUILD_FIXED_POINT_BOUND`, past which
  `Finding::BuildNotSettled` names the nets still changing), so the build
  snapshot equals the live engine's state before its first wake
  (`board/tests/build_fixed_point.rs`).

### Power domains

Power is **volts, not booleans**:

```rust
pub struct PowerState { pub volts: f64, pub ok: bool }
```

- `PowerOut` pins source their net at the voltage their part publishes
  (`PinHandle::drive` on the pin sets it, `release` lets it go; the declared
  idle drive is what the rail holds before the part publishes); those rails
  are terminals, and enter every cluster that reads them as constants (the
  MNA needs values, and chip models need the numeric AVDD for range checks
  like PGA common-mode).
- A power net with **no `PowerOut` source anywhere** (board or harness) raises
  a **`PowerNetUnsourced`** finding and presents as down — this is precisely
  the "AVDD unstrapped" failure mode.
- A down rail is a **released** terminal (a real buck or LDO output is
  high-impedance): its node floats, its loads read only what else reaches
  them, and a bench strap onto the net sources it without a fight. A part
  with an active output discharge drives 0 V and cites it.
- **No implicit net-name merging across boards.** Two boards both naming a net
  `GND` share nothing until a harness connects them — grounds included. An
  unreferenced ground is a finding, not an assumption.
- Harness endpoints may declare a power kind
  (`from = "P2EVAL.3V3", kind = "power(3.3V)"`) so bench rigs can source
  domains without a designed PCB.

### Analog clusters

Nets connected through passive primitives form **clusters**, extracted at
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
- **Nonlinear elements are branches**, never drives: a component declares
  them through `Component::branches()` (`Branch { a, b, curve, control }`,
  `NODES.md` §11) and a netlist part with no model through a `PwlSpec`
  (`register_pwl`). An element joins its two nets — and its control net —
  into one cluster, which then always solves; the engine chooses each
  element's region in a cold-started, ordered, bounded flip loop inside the
  solve (`board/src/cluster.rs`), never the component. A pin's current, from
  the same solve, is an instrument: `PinHandle::sense_current`,
  `ComponentNetIo::on_branch` (which escalates the cluster and nothing
  else — no floating-sense finding for an open loop, and the fight findings
  above still raised beside the operating point), and
  `BuiltSystem::branch_current` / `pin_current` by path.

```rust
pub trait ClusterSolver: Send + Sync {
    fn solve(&self, cluster: &Cluster, inputs: &ClusterInputs) -> ClusterSolution;
}
```

The trait is the deliberate seam: the default is `QuasiStaticMna`; a transient
SPICE-backed solver is a possible future implementation and is intentionally
**not** part of this design (no ngspice dependency, no cluster-marking syntax —
see the consumer decision record for the rationale and revisit trigger).

### Step clocks and serial (one drive type; serial is levels)

There is no second channel (`DESIGN.md` rule 2). A step clock is the third
encoding of the one per-instant message, `Drive::Periodic { hi, lo, segment }`
(`NODES.md` §10/§11, the §12 item 5 record): two Thevenin ports and the
integer `PeriodicSchedule` that alternates them — the half of the drive a
peripheral owns (`embsim_peripherals::pulse_out`), anchored and integrated in
the engine's nanoseconds. UART bytes are framed onto ordinary
digital pins as timed levels by
[`SerialLevelBridge`](board/src/serial_levels.rs) (codec in
`board/src/uart.rs`). There used to be `Producer`/`Consumer` byte-route roles,
a `board/src/stream.rs` pipe layer, and then a pulse channel with its own pin
roles, routing pass and delivery; they are gone.

**Step clocks** (`Drive::Periodic`):

- A step-clock pin is a plain digital output; a periodic drive on it is
  published **once per rate change** (start / retarget / stop) and resolved
  like any drive, **phase by phase** through rule 2: a pull follows the
  square wave, a comparable static source or a second clock is `Contention`
  with its finding. The net publishes `NetState::Periodic { hi, lo, segment }`
  and a consumer integrates the segment at read time — not per edge — once
  its two phases cross the consumer's own thresholds (`PeriodicSense::rate`);
  a self-biased input is the exception (`NODES.md` §12 item 5, the cleanup
  and the final pass).
- Across a coupling capacitor the rate crosses by the AC rule
  (`1/(2π·f·C) ≤ R_far / 10`, else `PeriodicNotCoupled`); a declared terminal
  is a barrier.

**Serial over pins** (levels, not a byte pipe):

- TX/RX pins are a plain push-pull output and a plain digital input.
  `SerialLevelBridge` clocks start/data/stop bits onto the net; the peer
  decodes levels back to bytes. Contention, floating lines, and series
  resistors are therefore ordinary net effects.
- Scenario byte-drop on a *route* (`stream_drop`) was deleted with the byte
  route. Level-domain fault injection for the serial era is
  [`Scenario::edge_fault`](https://github.com/RileyMcCarthy/embsim/pull/48)
  (draft; issue #44) — do not invent a byte-pipe injector here.

### Netlist ingestion

Input: KiCad s-expression netlist (`kicad-cli sch export netlist`). Parsing is
**version-gated** on `(export (version …))` — unsupported versions fail with a
named error, and the test suite carries one fixture per supported KiCad major.

Parsed per component: `ref`, `value`, `footprint`, `libsource (lib, part)`,
sheetpath (hierarchical designs), fields/properties; per net: `code`, `name`,
`(ref, pin, pinfunction?, pintype?)` nodes. `pintype` is the schematic
symbol's electrical type (`passive`, `power_in`, `…+no_connect`) — carried for
diagnostics only; electrical descriptors always come from the component's own
`PinDecl`, never from the schematic. Everything else a real export contains
(`design`, `libparts`/`libraries`, datasheet/description/fields, non-`dnp`
properties, `tstamps` UUID paths) is deliberately ignored — enumerated in the
`netlist` module docs.

**Net names are canonically the full exported name, sheet path included.** A
KiCad local label is scoped to its sheet instance and exports as
`/<sheet>/<label>`; global labels, power symbols, and root-sheet labels export
bare. The parser never strips the path: two sheets may carry the same leaf
label, so `/Sheet2/SIGNAL` and `/Sheet3/SIGNAL` are two electrically distinct
nets whose leaves collide, and stripping would give every name-keyed artifact
(scenario `net_stuck`, findings, dumps) a silent cross-sheet merge while the
graph kept them apart. `normalize_net_name` canonicalizes overline syntax on
the leaf only; `net_short_label` is the display-only shortening, explicitly
not an identity. The hierarchical reference fixture is the MaD EdgeBoard
export (3 sheets, 168 components, 243 nets, 53 sheet-scoped names).

Classification is **one pipeline** (`DESIGN.md` rule 1): netlist part →
registry class → node. Every netlist part gets a node of a class with
behaviour, or the board refuses to build naming the part and its value; there
is no stub list, no ignored tier and no allow-list. It is three-tier and
**keyed primarily on the libsource *part* name** — the lib name is best-effort
only (real exports contain empty lib names and KiCad `*-rescue` libs; rescue
mangling like `DS2_Addon-rescue::Jumper_NO_Small-Device` is normalized before
matching):

| Tier | Match | Result |
|---|---|---|
| auto | part `R*`/`C*`/`L*`/`LED`/`D_*` from `Device` (or rescue thereof) | passive primitive; value parsed from the **first token** of the field (`47R`, `4k7`, `0.1uF`, `4.7uF 6.3V`, `47uH/3A`); **pin-count validated** — a 2-terminal class with ≠2 pins is a hard classification error. A `LED`/`D_*` symbol **yields to a registry entry** (by part name, manufacturer part number or value): the symbol is a guess that the part is DC-open, an entry a statement about the purchasable part; with no entry it stays the DC-open passive |
| auto | `Conn*`/`Screw_Terminal*` parts, plus any part name the consumer passes to `PartRegistry::register_boundary` | board boundary pins (harness attachment points) |
| auto | `Jumper*` parts | stateful short; default from name (`_NO`/`_Open` → open, `_NC`/`_Bridged` → closed; 3-pin `Jumper_3_*` variants get a selectable position) |
| auto | two-pin `SW_*` parts | a **switch** with one open pole across pins `1` and `2` (the KiCad `Switch` library's two-terminal pinout) — unless the part name is registered: the pairing is a guess from the library convention, and an explicit registration beats it |
| auto | `TestPoint*` | one pin: a **probe** node (senses, never drives); no net: a **mechanical** node (a pad); more pins: whatever the registry says of the part name, else a pin-count error |
| auto | `MountingHole*`/`Logo*`/`Fiducial*` | a **mechanical** node: pads recorded, nothing electrical |
| registry | anything else, keyed by part name, then by the **manufacturer part number** the export carries (`Manufacturer_Part_Number`/`MPN`, `ComponentDecl::mpn`), falling back to `value` | a consumer-registered class: a `Component` constructor (`register`), a switch with declared poles by pin id (`register_switch`), a piecewise-linear element (`register_pwl`: a `PwlSpec` of pins and the branches between them — a diode's knee, a channel and its control — which the system build stamps into the cluster solve; `embsim_models::pwl_library` is the library of such entries with their datasheets) or a mechanical part (`register_mechanical`). One table: a later registration of a key replaces the earlier, whatever its kind |
| error | no registry match | `RegistryError::UnknownPart { reference, part, value }`; system construction fails |

A closed switch pole is a **build-time identity union** of its two pins' nets —
the merge `pin_short` makes — honouring `pin_detach` on either pin; an open pole
is nothing. Poles are set by `Scenario::switch(reference, pole, state)`, indexed
from 0 in declaration order; `Scenario::jumper` is the one-pole form (pole 0).

Active parts come from standard *and* custom libraries alike (`74xGxx`,
`Isolator`, `Interface`, `Transistor_BJT`, `Switch` are all standard-lib
actives on real boards) — tier 2 is "whatever tier 1 does not match," not
"custom-library symbols."

**Netlists with no libsource.** Every tier above keys on the part name, which an
EDA export always carries — but a whole-module netlist transcribed from a vendor
schematic PDF has no symbol library to name, so every part name is empty and the
entire board lands in the error tier. `PartRegistry::classify_unnamed_by_reference`
opts such a board into a narrow fallback: **only** when a component's part name
is empty, the auto tiers match a class synthesized from its reference-designator
prefix (`R`/`C`/`L`/`D` → the passive primitives, `J`/`P` → boundary, `TP` →
probe, `H`/`MK` → mechanical), and the whole leading alphabetic run must match,
so `RN7` and `PCB` still fall through. Active-silicon prefixes (`U`, `IC`, `Q`,
`X`, `S`, …) deliberately map to nothing: the fallback classifies what the engine
already knows how to be, and refuses to guess the rest, which then registers by
`value` — and an explicit registration for a value beats the synthesized class,
so a `J`-prefixed solder link registers as the switch it is and a `J`-prefixed
mounting hole as mechanical. It is opt-in because an empty part name in a real
export means a damaged export, not a naming convention. Reference fixture:
`board/tests/fixtures/p2_ec32mb.net` (Parallax P2-EC32MB module, 114
components, zero part names — 85 passives and 2 pad/socket symbols classify, 2
switches, 4 mechanical parts and 21 active parts register).

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
- `switch("Board.S1", pole, Closed)` — a switch pole, by index: the same
  union, said as the position it is (`jumper` is the one-pole form);
- `net_stuck(net, rail)` — add a Thevenin source to a net;
- `value_override("Board.R5", "4k7")`, `dnp_override("Board.C7", Populated)` —
  scenario-time BOM changes;
- Level-domain serial/line faults: see draft PR #48 / issue #44
  (`Scenario::edge_fault`) once merged — not a byte-route `stream_drop`.

Harness endpoints are `Board.Connector.Pin` references; bare MCU-pin endpoints
(`P2EVAL.P0`) are allowed for bench rigs that aren't a designed PCB —
`System::component(name, component)` registers such a **bench component**
(no board netlist; each declared pin becomes a `{name}.{pin}` net addressable
as a bare endpoint, with full electrical descriptors).
Deliberately wrong harnesses (swapped pins) are valid fixtures — the
`Contention`/`Floating` findings are the assertion targets.

## The MCU as a component

A platform crate (per `CONTRACT.md`) provides the MCU component:

1. **Firmware image**: the consumer's static library. The engine **spawns**
   the entry on a component-owned thread: `McuBuilder::entry` puts the
   component in owned-execution mode, and `Component::start` — called by
   `System::start` strictly after every component has attached — spawns the
   entry bound to the component's own `PeripheralInstance`. CONTRACT.md's
   init-ordering section is re-stated for the board engine: `attach()` and
   net service are live before the firmware entry's first instruction, and
   peripheral-bank sizing performed *inside* the entry commutes with the
   bridges attach installed (`serial::init` preserves installed FDs).
   Entry-less components stay in facade mode: `Emulator::run` on the
   caller's thread against the default instance keeps working unchanged.
   Inside a P2 package (`embsim_boards::p2::P2Package`) the core's start
   is gated once more, by the chip: the package holds `P2Core::start` —
   the firmware entry, a QEMU core's first wake — and every wake the core
   asks for until the datasheet's 3 ms restart delay has run out after
   `RESN` read released with `VDD` inside its window, and starts it then,
   on the engine thread, in a wake of its own (`NODES.md` §2 "MCU node
   (P2)", the START gate). A core's wakes reach the engine through the
   package's `WakeGate` whichever way it schedules — through `P2Pads`, or
   on the net I/O it was handed (`ComponentNetIo::with_wake_gate`) — so the
   native firmware image is held like any other core; its bridged pads
   publish nothing before START and drive at their bank's supply after it
   (`McuComponent::host_pads`).
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

Channel behavior stays HAL-granular (GPIO levels, pulse rates, and — where a
bridge still uses the peripheral serial bank — socketpair byte FDs); pins are
topology. Baud and channel parameters come from the same tables — the
emulator stops inventing its own defaults (consumers may keep explicit pacing
overrides for tests). Level-framed serial (`SerialLevelBridge`) puts bits on
the pin facade directly rather than through a derived byte route.

> **Slice status (2026-08):** the MCU-as-a-component pattern ships in
> `board/src/mcu.rs` for **all four channel kinds**, each opt-in per channel
> through `McuBuilder`: serial (socketpair bridges into the peripheral serial
> bank, baud from the table), GPIO (**bidirectional** — firmware writes drive
> the net, external drives sense back into the bank, both honouring the
> table's `active_low`), pulse-out (one STEP pin, below), and encoder (a
> quadrature pin pair ×4-decoded into the bank as *increments*, so firmware
> homing re-bases rather than being overwritten). The entry inversion in
> point 1 is **delivered**: `McuBuilder::entry` + `Component::start` spawn the
> firmware on a component-owned instance (facade mode without an entry keeps
> the `Emulator::run` flow working). Point 3's table read path is
> `embsim-memory-inspect`'s `hal_tables` module (symbol names parameterized;
> the reference consumer's names are the documented defaults).
>
> **A step clock is a rate on the wire, not edges.** Channel behavior stays
> HAL-granular everywhere else, but a pulse-out channel is the one signal
> whose edge count runs orders of magnitude ahead of the rest of the board: at
> the reference machine's 8192 steps/mm, one mm/s is 8192 edges/s, each of
> which would be a drive, a cluster resolution and a sense delivery through
> the single-writer engine. So the STEP pin carries a `Drive::Periodic` —
> the pad's high and low ports around the peripheral's `PeriodicSchedule`
> (frequency, accumulated count, ceiling and an anchor nanosecond) — published
> **once per rate change**, resolved on the net like any drive, and
> integrated by the consumer at *read* time, the discipline
> `DETERMINISM.md` mandates. Counts stay exact: `PeriodicSchedule::emitted_at_ns`
> is the same integer arithmetic `HAL_pulseOut_run` hands the firmware; the
> wire carries no direction (the drive reads its own DIR pin). Measured: a
> four-segment motion profile delivering 65 536 pulses costs **46 engine
> events** (52 with the rig's drive keeping its own step counter — two more
> subscriptions, six more deliveries), unchanged as the pulse count moves by
> tens of thousands (`board/tests/pulse_bridge.rs`). The fidelity this trades away — no edges,
> no pulse width, no per-edge DIR sampling — is enumerated on
> `Drive::Periodic`.

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
`CurrentIntoFloatingNode`, `NonConvergent` (an element cluster's region loop
ran its bound without a consistent set of regions: its nodes float and the
finding names the elements), `PowerNetUnsourced` (a power pin on a net no
source reaches — or one that floats behind an off element, a rail blocked by
a reversed polarity FET), `PeriodicNotCoupled` (a periodic drive's rate a
coupling capacitor's reactance refuses), `ClassificationError`, `UnconnectedRegistryPin` (both directions:
declared-but-absent and present-but-undeclared), and the five the build
raises after its fixed point from the settled states and the parts'
declarations (`NODES.md` §8 phase 4, §12 item 5): `RailDown` (a `PowerOut` pin whose
net floats, with the reason the build can see — an input unsourced, the
output's reference unheld, or the part holding it, a soft-start the
snapshot is early for), `UnreferencedDomain` (a pin whose net is sourced
while its declared reference's net is not — an isolator's unwired
secondary ground), `UndecoupledPowerPin` (a power-in pin with no capacitor
to its reference), `MechanicalOnDrivenNet` (a mounting hole's pad on a net a
pin drives), `OpenDrainWithoutPullUp` (an open drain — a signal pin that
sinks and cannot source — whose net no pull-up reaches through the
resistive network: no rail, supply above 0 V, sourcing pin or input port
biased above 0 V; asked of the declarations and the network, not of the
settled states, so a rail still in its soft-start counts; an open drain
whose net reaches no other part's pin — a no-connect, a net that leaves the
board only through a connector — raises nothing).

## Testing conventions

- Parser: committed netlist fixtures per supported KiCad major (hand-written
  minimal + one real exported board) with golden component/net graphs.
- Net resolution: truth-table tests per rule (driver combinations × expected
  `NetState`), including the weak-drive and impedance-escalation boundaries
  (`engine::tests::source_strength`).
- MNA: hand-computed reference circuits (bridge, divider ladder, pull-up vs
  driver, source-free singular cluster) asserted to µV.
- Pulse routes: routing through series passives, facing-`PulseSource`
  detection, route invalidation on jumper/fault changes; serial-as-levels
  bit clocks (`board/tests/serial_levels.rs`, determinism `serial_levels`).
- Pin bridges: fake firmware driving the peripheral free functions with peer
  components watching the pins — exact step counts, mid-train direction
  reversal, GPIO in both directions at the channel's polarity, encoder counts
  arriving, an asserted engine-event ceiling at a realistic step rate, and a
  stepped-mode N-run identity over all of it
  (`board/tests/{pulse_bridge,pulse_bridge_stepped,carriage_seam}.rs`).
- System: a two-component smoke board (fake MCU pin driver + fake sensor)
  exercising attach/schedule/diagnostics without any consumer firmware.
- Whole machine: the reference consumer's three real boards — a vendor MCU
  module (`p2_ec32mb.net`), the carrier (`mad_edge.net`) and a sensor add-on
  (`ds2_addon.net`) — assembled through harnesses with the machine's motor,
  encoder and end switches, asserting build findings, the card-edge
  correspondence, and a command/reply byte exchange along the whole serial path
  (`board/tests/{ec32mb_module,edgeboard,machine_system}.rs`, with the part
  library and every classification/harness decision in
  `board/tests/machine_parts/mod.rs`).

## Non-goals

- SPICE/transient analog simulation (trait seam reserved; not built).
- Cycle-accurate MCU peripheral timing; emergent RX-FIFO overrun on a
  deleted byte pipe (serial is levels now; line faults are net effects —
  level-era injectors land via #44 / PR #48, not a scenario byte-drop).
- PCB physical effects (parasitics, thermal, EMC).
- Auto-generating *plant* physics — transducer components expose parameterized
  primitives (e.g. bridge legs) for consumer physics models to drive.
