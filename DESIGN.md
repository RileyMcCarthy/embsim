# embsim — the design, and the rules that keep it

This is the document to read before proposing a change to the engine, a
model, or the interface between them. It says what embsim is, the rules
every part of it obeys, the limits it holds to on purpose, and the test a
proposal has to pass before it is in scope. The plan that carries the design
out is [`NODES.md`](NODES.md); the engine's internals are
[`BOARD_ENGINE.md`](BOARD_ENGINE.md); the determinism contract is
[`DETERMINISM.md`](DETERMINISM.md); the firmware ABI is
[`CONTRACT.md`](CONTRACT.md).

## 1. What embsim is

An **event-driven, quasi-static electrical board simulator that runs the real
firmware on an instruction-set simulator.** A board is its vendor netlist;
every part on it is a node; the nets between nodes are resolved as circuits
at the instants things happen; time is a stepped virtual clock that advances
only to the next scheduled instant.

It validates a PCBA with its firmware before the board exists, in CI, on
every push. It sits between a unit test with a mocked HAL and the bench. It
is **not SPICE** (no timestep, no transient analysis) and **not a
transaction-level emulator** (no bytes handed across a bus; levels on wires).
Being neither is the point: it sees the bugs the two ends cannot — a fought
line, a missing pull-up, a floating input, a strap on the wrong pin, a shared
bus clobbering itself, a boot that needs a switch position — at a per-edge
cost that lets a whole boot run in a fifth of a second.

## 2. The model, in one paragraph

Every part contributes to one quasi-static nodal solve. Resistors, closed
switch poles and on-state channels are conductances; rails are declared
terminals, sources referenced to their own ground pin, entering every
dependent solve as constants and forming cluster boundaries; pads are
Thevenin sources at the device's real strength; diodes, LEDs, FETs and BJTs
are piecewise-linear elements whose region is chosen by a bounded, ordered
flip loop inside one solve, cold-started every time; capacitors are
single-pole closed forms. A solve runs only when an input to a cluster
changes and yields the operating point at that instant. Time enters in
exactly one way: a node with capacitance, or a rail with a soft-start,
publishes its closed form, and the instants at which a receiver would flip
are computed and armed on the wheel as integer nanoseconds. Nothing is
integrated per tick.

## 3. The rules

Each rule names what enforces it. A change that needs to break a rule is a
change to this document first, with the measurement that justifies it.

**Rule 1 — One pipeline.** A netlist part is a node whose class has
behaviour, or the board refuses to build and names the part and its value.
Netlist part → registry class → node. No stub tier, no facade-only part, no
allow-list, no "declare an output as an input so it cannot contend". A
connector is a pass-through node, a test point a probe node, a mounting hole
or board outline an explicit mechanical node, an IC nobody has modelled a
build error.
*Enforced by:* `RegistryError::UnknownPart { reference, part, value }`; the
`stub_count` census test, which must read 0 on every board.

**Rule 2 — One interface.** Between a node and the engine there is exactly
this: static facts declared once on `PinDecl` (idle drive, clamps, input
port, capacitance, thresholds with hysteresis, reference and supply pins,
`can_source`) and on `Component::branches()` (nonlinear elements as branches
between two of the part's pins); one per-instant message, `Drive`, with three
encodings — `Thevenin { volts, ohms }`, `Current { amps }`, `Periodic { hi,
lo, segment }` — all sequenced through one command; one delivery, `Sense {
volts: Option<Volts> }`, relative to the pin's reference, `None` meaning no
source reaches the node; the receiver's own projection to a level through its
declared thresholds; and wake scheduling. No open-drain or push-pull kinds:
open-drain is a sink that releases. No second channel: the pulse train is an
encoding of `Drive`, kept because 820 000 edges a second was measured and
refused. Current into a pin is an instrument, not the normal path.
*Enforced by:* the API surface itself — there is no other way to reach a
net — and the facade check that refuses a pin the netlist does not have.

**Rule 3 — Publish at your own instant.** A producer publishes what it
drives on its own pin, at the instant it drove it, never the net's result.
A consumer reads a quantity from the drive it is handed, never by
differencing timestamps. Retraction is a superseding publish. No node
commits state ahead of virtual time; it schedules a wake and acts when the
engine arrives. The engine never advances past an undelivered publish.
*Enforced by:* `transition_fidelity`, the scope on the flash clock in
`rom_boot_ec32mb` (every edge at its own nanosecond, none shared), and the
determinism goldens.

**Rule 4 — The engine owns resolution.** A node never sees another node,
never reads a net it has no pin on, and never resolves anything. The engine
alone ranks sources by total ohms (a pull never contends; a source ten times
weaker loses with a finding; comparable sources solve), runs the nodal
solve, arms RC crossings, chooses PWL regions, and reports contention and
floating as findings. Three structural rules keep that tractable: terminals
are declared and are cluster boundaries; source-strength projection has one
form; cluster membership is fixed at build and the only runtime-mutable
conductance is the plant's.
*Enforced by:* the incremental-versus-full resolution property test, the
cluster census bound (m ≤ 8 on every board), and the fact that no node API
can union nets or read another net.

**Rule 5 — No timestep, ever.** Time enters only as scheduled instants and
closed forms: a single-pole RC, a linear ramp, a periodic segment. Regions
are chosen inside one DC solve. The refusals are part of the rule:
capacitors are never cluster edges (that is a multi-pole transient by the
back door); no Newton iteration or exponential device laws; no capacitor on
a node that hosts a nonlinear element; no per-edge step trains; no inductor
dynamics. Anything that needs integration per tick is out of scope, and the
`ClusterSolver` seam stays the one place a future decision could put it.
*Enforced by:* review against this document, and the per-edge wall-time
budget as a CI gate.

**Rule 6 — Nothing invented.** No ground, rail voltage, pull-up, parasitic,
threshold, tolerance or drive strength that the netlist, a datasheet, or a
scenario line does not name. An unresolvable net is *no voltage*, never a
guessed level: `level_of` returns `None` and the receiver's declared policy
decides. Ground is a declared terminal. A model's every number carries its
provenance (BOARD_ENGINE.md, "Model provenance convention").
*Enforced by:* `FloatingSense`/`Contention` findings, provenance review, and
the rule that a default threshold is a named constant with a citation.

**Rule 7 — Deterministic and reproducible.** Same binary, same inputs, same
bits. Instants are integer nanoseconds; the RC log is embsim's own
fixed-point function, not a platform `ln`; no hash-ordered iteration on the
resolution path; every armed instant carries its solve generation and fires
in (deadline, sequence) order. Golden traces must pass without re-blessing
unless the phase that changes them says so and reviews the diff.
*Enforced by:* the determinism job and its goldens.

**Rule 8 — Fast by construction.** Nothing on the fast path pays for a
feature it does not use. A solve runs only where sources within a factor of
ten disagree or an analog sense asks; everything else is a projection. A
root with one reaching source delivers its open-circuit voltage without a
solve. Step trains travel as rates. Measured budgets are CI gates, and a
feature that regresses them is not done.
*Enforced by:* the ROM-boot edge count and wall time, the solve benchmark,
and the cluster census.

**Rule 9 — Every model has provenance and a proving test.** A node's numbers
cite a datasheet; a phase is done when an observable assertion says so, not
when "it works". Tests assert what the board *did* — where the flash was
read, which instant an edge landed on — not that a run finished.
*Enforced by:* review, and the phase list in `NODES.md`, each with its
proof.

## 4. Scope

What it validates, what it does not model, and the path from today to the
plan are in the README's "What embsim is for, and what it is not". In one
line each: connectivity and design intent; every DC operating point;
protocols bit by bit on the real wires; timing at the event level; the
firmware whole; what-ifs by scenario. Not: signal integrity, transients
beyond one pole, nonlinearity beyond regions, tolerances, power integrity,
noise, cycle-exact CPU timing, faults nobody injects, EMI/ESD/mechanical.

## 5. The scope test

A proposal is in scope when every answer is yes:

1. **Is it a projection or a closed form armed at an instant?** If it needs
   integration per tick, no.
2. **Is every number it needs named** by the netlist, a datasheet, or a
   scenario line? If it must invent one, no.
3. **Does it go through the one interface** — declarations, `Drive`,
   `Sense`, a wake? If it needs a second channel or a shortcut past the
   engine, no.
4. **Does it keep the fast path free?** If a node that does not use it pays
   for it, no.
5. **Does it keep one pipeline?** If it needs a stub, a facade or an
   allow-list, no.
6. **Is there an observable that proves it**, and do the existing goldens
   still pass without re-blessing? If not, not yet.

A "no" is not a veto; it is a decision to write down in `NODES.md` with the
measurement that justifies it, before the code.

## 6. Changing this document

These rules were set after measuring the alternatives: the cost of an edge
(0.61 µs), of a cross-thread synchronisation (10.9 µs), of a step train as
edges (820 000 a second), of a boot through the net engine (16 901 edges in
0.2 s), and after three independent designs for making every part a node
were judged and challenged. A rule changes when a new measurement says the
trade-off moved, and the change is a commit to this file with that
measurement in its message.
