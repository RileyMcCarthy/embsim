---
applyTo: "**"
---

Rust (embsim SIL framework). Overview: `README.md`; platform ABI contract:
`CONTRACT.md`.

- Generic crates stay **project-agnostic** — no consumer-specific constants,
  names, or paths. Consumer wiring belongs in the consumer's repo.
- Scrutinise FFI `unsafe` boundaries: guard-before-deref (null/len/sign) on
  every trampoline; symbol names verbatim from the firmware's HAL headers.
- The tree is rustfmt-formatted and `cargo fmt --check` gates CI.
- Public items need `///` docs; `cargo doc` runs with `-D warnings` in CI.
- Peripheral state is process-global by design (one firmware per OS process) —
  tests that touch shared globals serialize behind the crate's `TEST_LOCK`.

Don't re-report Clippy or build errors.
