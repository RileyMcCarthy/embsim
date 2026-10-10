# The host's Chrome on a serial line

This directory is an embsim project with no board: a page in the host's
Chrome writes to its Web Serial port, and the port is a `chrome-cdp`
component's serial line, wired to a `host-serial` component on one 3.3 V
rail ([`PROJECTS.md`](../../PROJECTS.md) §5). It shows the kind with
nothing else in the way: the page's clock lives the board's time, a
millisecond at a time over the Chrome DevTools Protocol, and its bytes
cross the line as levels.

| File | What it is |
|---|---|
| [`project.toml`](project.toml) | the two components, their wires and the rail |
| [`ping.html`](ping.html) | the page: opens the port the scenario grants it and writes `ping` every 100 ms of its own clock |
| [`harness.mjs`](harness.mjs) | a Playwright harness that attaches to the run's Chrome and reads the page in the page's own time |

## Running it

From this directory, with `embsim` installed (`cargo install --locked
--path cli` from the embsim checkout) or run in place (`cargo run -p
embsim-cli --`), and Google Chrome (or Chromium) on the host:

```bash
embsim check project.toml        # starts nothing: Chrome is found, the page is a file
embsim run project.toml --for 1s
```

Chrome is launched, headless, at the run's first slice, with the board's
clock held while it comes up; the page opens once Chrome holds it. A second
of board time is ten periods of the page's 100 ms timer from the moment
its port opens, a few milliseconds in, so the run hears nine pings, 45
bytes, however long the second takes on the host. The run's summary says
how many slices it took, what the page lived of the board's time (all of
it), and the host time per slice. A program on the host reads the pings
from the PTY, `.embsim/HOST.pty`.

To drive the page from a harness, run for longer and attach once the run
prints where Chrome's DevTools are (its "reached" line):

```bash
embsim run project.toml --for 30s
node harness.mjs http://127.0.0.1:PORT
```

`cdp/tests/real_chrome.rs` runs this project with the shipped kinds, in
process, and checks the nine pings and the page's clock against the
board's (CI's `chrome-cdp` job).
