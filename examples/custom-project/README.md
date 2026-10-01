# A project with kinds of its own

This directory is an embsim project whose boards need things embsim does
not ship, and that adds them itself: a catalog crate of its own, named in
the project file and built into the `embsim` command by the tool
([`PROJECTS.md`](../../PROJECTS.md) §10). It adds one kind of each sort:

| Kind | Sort | What it is | Source |
|---|---|---|---|
| `example-ex-buf1` | part model | the EX-BUF1 single Schmitt-trigger buffer | [`catalog/src/buffer.rs`](catalog/src/buffer.rs) |
| `example-buffer-board` | board | a small board: the buffer behind a four-pin header, with pull-downs on its input and output | [`catalog/src/board.rs`](catalog/src/board.rs), [`catalog/netlists/buffer-board.net`](catalog/netlists/buffer-board.net) |
| `example-blinker` | P2 core | a core that toggles one pad on a schedule, as a three-instruction program would | [`catalog/src/blinker.rs`](catalog/src/blinker.rs) |
| `example-edge-counter` | bench component | an instrument that counts rising edges on its input | [`catalog/src/counter.rs`](catalog/src/counter.rs) |

**The EX-BUF1 is an example part.** It does not exist. Its datasheet,
[`catalog/datasheets/EX-BUF1.md`](catalog/datasheets/EX-BUF1.md), is a
stand-in written for this example so that the model can cite a section for
every figure it uses, the way embsim's own models cite their vendors'
datasheets. A project's real model cites its real part.

## The system

[`project.toml`](project.toml): a P2-EC32MB module (a board embsim ships),
powered from its edge fingers as a carrier powers it, runs `example-blinker`
in its processor's package, toggling `P0` every half millisecond from the
moment the processor leaves reset. `P0` drives the buffer board's input;
the board takes its supply from the module's own I/O rail on its `V00`
finger and its ground from the module's; the buffer's output goes to the
edge counter.

```text
EC32.J203.40 (P0) ──► BUF.J1.1 (A)   EX-BUF1   BUF.J1.2 (Y) ──► COUNTER.IN
EC32.J203.47 (V00) ─► BUF.J1.3 (VCC)
EC32.J203.45 (GND) ─► BUF.J1.4 (GND) ◄──────────────────────── COUNTER.REF
```

## Running it

From this directory, with `embsim` installed (`cargo install --path cli`
from the embsim checkout) or run in place (`cargo run -p embsim-cli --`):

```bash
embsim check project.toml
embsim run project.toml --for 10ms --net BUF.OUT
```

The first `check` builds the runner: `embsim` writes a small crate into
`.embsim/runner-<id>/` that depends on embsim and on `catalog/`, builds it
with Cargo in the release profile, and runs the project through it. The
crate is a member of embsim's workspace, so the runner builds in embsim's
`target/` and reuses what the workspace built. After that a `check` or a
`run` with nothing changed costs Cargo's no-op check; edit the crate and
the next one rebuilds just it. `check --rebuild` starts the runner afresh.

The run prints, among the module's own findings and reports:

```text
[   5.600000 ms] EC32.U100: blinker: P0 high at 5.500000 ms, flipping every 0.500000 ms
[   5.600000 ms] COUNTER: rising edge 1 on IN at 5.500012 ms
[   6.600000 ms] COUNTER: rising edge 2 on IN at 6.500012 ms
…
COUNTER: 5 rising edges on IN, the first at 5.500012 ms and the last at 9.500012 ms, every 1.000000 ms
net BUF.OUT: Driven(High)
```

The processor leaves reset at 5.5 ms (the module's bucks' soft-start, then
the P2 datasheet's 3 ms restart delay); each edge reaches the counter the
buffer's 12 ns propagation delay after the pad flips.

## The crate

[`catalog/`](catalog) is an ordinary library crate:

- [`src/lib.rs`](catalog/src/lib.rs) holds the registration function,
  `pub fn register(set: &mut CatalogSet) -> Result<(), ProjectError>`, which
  the runner calls: it adds the board, part and component kinds as one
  `Catalog` and the core as a `CoreCatalog`.
- Each kind is in a module of its own, each model's numbers with their
  citations.
- [`tests/project.rs`](catalog/tests/project.rs) runs `project.toml` in
  process, as the runner runs it (`embsim_cli::run_with_crates`), and
  asserts the instant of every edge: how a project tests its own catalog
  without building a runner.
- [`examples/own_binary.rs`](catalog/examples/own_binary.rs) is the same
  command as a binary of the project's own, for a project that would
  rather own it than have the tool build a runner: `embsim_cli::shipped()`,
  the registration function, `embsim_cli::main_with`. From this directory,
  `cargo run -p custom-project-catalog --example own_binary -- run
  project.toml --for 10ms` prints what `embsim run` does.

`embsim new --catalog DIR` starts a crate of this shape for a project of
your own, with one commented example of each sort of kind to keep, rename
or replace.
