# embsim-cpu-oracle

Parse and diff **silicon goldens** for an instruction-set simulator.

Silicon runs a program and prints observations. Those lines are the spec.
The ISS must emit the same text. This crate does not know any particular
ISA: a CPU adapter compiles the program, loads the chip, and runs the ISS.

Two on-disk layouts:

1. **Report** — `PASS name <hex>`, `FAIL …`, `RESULT 0 PASS` (named checks).
2. **Records** — `CASE` / `IN` / `OUT` maps (one-instruction probe).

```rust
use embsim_cpu_oracle::{diff_records, parse_records};

let silicon = parse_records(&std::fs::read_to_string("golden/probe.txt")?)?;
let iss = /* adapter: run each CASE.input on the ISS, collect OUT */;
let misses = diff_records(&iss, &silicon);
assert!(misses.is_empty());
```

Capture (loader, serial, compiler) stays with the CPU adapter. The P2
adapter lives in MaD `SIL/p2core/hwtest/` and `tools/hw_probe.py`.
