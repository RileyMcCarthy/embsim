# The Chrome guest: building its image

A Debian image that boots straight into a headless Chromium, for the
`chrome-vm` bench component (`PROJECTS.md` §5) and
`embsim_qemu::ChromeGuest`. It is built once with `build.sh`, cached outside
the repository, and never written to afterwards: every launch boots a
throw-away copy-on-write overlay of it, so a run cannot change the next.

## What you need

- The host's own system QEMU for the host's architecture, and `qemu-img`:
  `brew install qemu` on macOS (an Apple-silicon Mac builds and runs an
  `aarch64` guest under HVF); `apt install qemu-system-arm qemu-efi-aarch64
  qemu-utils` or `apt install qemu-system-x86 qemu-utils` on Debian or
  Ubuntu (KVM when `/dev/kvm` is writable, TCG otherwise). This is not
  `qemu-system-p2`, the P2's program `embsim qemu install` builds: the guest
  is an ordinary computer.
- For an `aarch64` guest, UEFI firmware: Homebrew's QEMU carries
  `edk2-aarch64-code.fd`; Debian's is `qemu-efi-aarch64`. `build.sh` and
  the node look beside QEMU and in the usual places, or take `FIRMWARE=`
  (`firmware` in a project).
- `curl`, `python3` (it serves the cloud-init seed to the guest), and
  `sha512sum` or `shasum`.
- About 4 GB of disk: the ~400 MB Debian base image and the ~3 GB result,
  both in the cache directory.
- Network access for the build, once: the base image from
  `cloud.debian.org`, and the guest's packages from Debian's mirrors. A run
  needs none.

## Building it

```bash
qemu/guest/chrome/build.sh       # 4 min on an M2 under HVF, the download included; much longer under TCG
```

It writes `chrome-debian13-<arch>.qcow2` to the cache —
`$EMBSIM_QEMU_CACHE`, else `$XDG_CACHE_HOME/embsim/qemu`, else
`~/.cache/embsim/qemu` — where `chrome-vm` and the tests look for it
(`embsim_qemu::default_image`), and prints its path. The steps:

1. Download Debian 13's `genericcloud` image for the architecture, once, and
   verify it against Debian's `SHA512SUMS`.
2. Boot an overlay of it under the host's accelerator with the cloud-init
   seed in `user-data`, served over HTTP on a free loopback port (the
   guest reaches the host at `10.0.2.2`, QEMU's user-mode network).
3. cloud-init installs Chromium, Python and the full kernel, writes the
   services and the policy below, and powers the guest off. The build waits
   for that (`BUILD_TIMEOUT`, 40 minutes by default) and checks the console
   for `EMBSIM-PROVISIONED`.
4. Flatten the overlay into one standalone qcow2.

Environment: `ARCH` (`aarch64` or `x86_64`, default the host's), `OUT` (the
image's path), `EMBSIM_QEMU_CACHE`, `QEMU` and `QEMU_IMG` (the binaries),
`FIRMWARE` (aarch64 UEFI), `ACCEL` (`hvf`, `kvm` or `tcg`),
`BUILD_TIMEOUT` (seconds).

`build.sh` is idempotent: it reuses the downloaded base image and overwrites
the output. Change `user-data` and rebuild; the image carries no state from
a previous build. A failed build prints the end of the guest's console.

## Using it

In a project, the host's end of the board's serial link is a `chrome-vm`
in place of a `host-serial`, on the same four pins:

```toml
[[component]]
name = "HOST"
kind = "chrome-vm"
[component.options]
baud = 2000000
devtools_port = 9222      # optional: a free port is picked otherwise
```

`embsim run` boots the guest at its first slice, one quantum after the run
starts, with the board's clock held there while it does (about ten seconds
on an M2), and prints where DevTools is. A harness attaches with
Playwright's `connectOverCDP("http://127.0.0.1:9222")` — it never launches a
browser — and opens the app, served on the host, at `http://10.0.2.2:<port>`.
From then on the guest, its Chrome and the page's workers run only while the
board's clock advances. The page's Web Serial sees an FTDI FT232
(`usbVendorId` `0x0403`), the board's serial line.

The tests that run it are `#[ignore]`d, as they need the image; one of
them is that round trip, a page served from the host trading bytes over Web
Serial with a `host-serial` port on the board:

```bash
cargo test -p embsim-qemu --test chrome_guest -- --ignored --nocapture
```

## What listens where

| inside the guest | what | reached from the host as |
|---|---|---|
| `127.0.0.1:9222` | Chromium's DevTools (`--remote-debugging-port`) | — (loopback only; Chromium ignores `--remote-debugging-address`) |
| `0.0.0.0:9223` | `embsim-devtools-relay` → 9222 | `hostfwd` to the host port `devtools_port` names (`embsim_qemu::GUEST_DEVTOOLS_PORT` is 9223) |
| the virtio-serial port named `embsim.agent` (`/dev/vportNpM`; resolved via sysfs) | `embsim-agent`: `t <seq>` → `<seq> <CLOCK_MONOTONIC ns>` | the `agent.sock` chardev; `Guest::clock_ns`, which books every slice from the guest's own clock |
| `/dev/ttyUSB0` | the emulated FTDI the board's serial line arrives on | the `serial.sock` chardev; `QemuNode` |
| `10.0.2.2` | the host, under QEMU user-mode networking | serve the web app there (`vite --host`) |

Chromium runs as the unprivileged `embsim` user (in `dialout`, so it may open
the port). The managed policy in `/etc/chromium/policies/managed/embsim.json`
grants pages served from the host every serial port without the picker, and
waives the secure-context requirement for them, since `10.0.2.2` is neither
`localhost` nor HTTPS and Web Serial is otherwise absent. Both policies match
explicit origins only (a bare host or `:*` is ignored, measured), so the
image lists the common dev-server ports: 5174, 4173, 5173, 3000, 8000, 8080.
Serve the app on one of those, or add yours to `user-data` and rebuild; the
app's own report that Web Serial is missing is the symptom of a port the
policy does not list.

## What is deliberately missing

- **Time from outside.** `systemd-timesyncd` is disabled: the guest's clock
  is the board's clock, metered by the node, and nothing may correct it.
- **Background work.** `apt-daily` and `unattended-upgrades` are disabled.
- **The cloud kernel.** Debian's `-cloud` kernel has no USB stack (no xHCI,
  no `ftdi_sio`); the image carries `linux-image-<arch>` instead so the FTDI
  enumerates.
- **IPv6.** The launcher turns it off on the guest's network: slirp's router
  advertisements land an address on the interface minutes after boot, and
  Chrome aborts every request in flight on the network change.

`root`'s console password is `embsim`, for debugging a guest by hand
(`-serial stdio` on a copy). A launched guest logs its console to
`console.log` in its VM's working directory. The guest has no route to
anything but the host.
