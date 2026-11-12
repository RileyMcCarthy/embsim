---
applyTo: "**"
---

Rust (embsim SIL framework). Overview: `README.md`; the design and its rules:
`DESIGN.md`.

- Generic crates stay **project-agnostic** — no consumer-specific constants,
  names, or paths. Consumer wiring belongs in the consumer's repo.
- The tree is rustfmt-formatted and `cargo fmt --check` gates CI.
- Public items need `///` docs; `cargo doc` runs with `-D warnings` in CI.
- The virtual clock is process-global — tests that init or re-anchor it
  serialize behind the crate's `TEST_LOCK` or live in their own test binary
  (`TESTING.md` rule 5).

Don't re-report Clippy or build errors.
