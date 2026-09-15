# The Chrome guest

A Debian image that boots straight into a headless Chromium, for
`embsim_qemu::ChromeGuest`. Built once with `build.sh`, cached outside the
repository (`~/.cache/embsim/qemu/chrome-debian13-<arch>.qcow2`), and never
written to afterwards: every launch boots a throw-away overlay of it.

```
guest/chrome/build.sh            # ~10 min under HVF/KVM; downloads ~400 MB once
EMBSIM_CHROME_IMAGE=... cargo test -p embsim-qemu --test chrome_guest
```

## What listens where

| inside the guest | what | reached from the host as |
|---|---|---|
| `127.0.0.1:9222` | Chromium's DevTools (`--remote-debugging-port`) | — (loopback only; Chromium ignores `--remote-debugging-address`) |
| `0.0.0.0:9223` | `embsim-devtools-relay` → 9222 | `hostfwd` to the port in `DevTools::port` |
| the virtio-serial port named `embsim.agent` (`/dev/vportNpM`; resolved via sysfs) | `embsim-agent`: `t <seq>` → `<seq> <CLOCK_MONOTONIC ns>` | the `agent.sock` chardev; `Guest::clock_ns` |
| `/dev/ttyUSB0` | the emulated FTDI the board's serial line arrives on | the `serial.sock` chardev; `QemuNode` |
| `10.0.2.2` | the host, under QEMU user-mode networking | serve the web app there (`vite --host`) |

Chromium runs as the unprivileged `embsim` user (in `dialout`, so it may open
the port). The managed policy in `/etc/chromium/policies/managed/embsim.json`
grants pages served from the host every serial port without the picker,
and waives the secure-context requirement for them, since `10.0.2.2` is
neither `localhost` nor HTTPS and Web Serial is otherwise absent. Both
policies match explicit origins only (a bare host or `:*` is ignored), so the
image lists the common dev-server ports: 5174, 4173, 5173, 3000, 8000, 8080.
Serve the app on one of those, or add yours to `user-data` and rebuild.

## What is deliberately missing

- **Time from outside.** `systemd-timesyncd` is disabled: the guest's clock
  is the board's clock, metered by the node, and nothing may correct it.
- **Background network.** `apt-daily`, `unattended-upgrades` are disabled.
- **The cloud kernel.** Debian's `-cloud` kernel has no USB stack; the image
  carries `linux-image-<arch>` instead so the FTDI enumerates.

`root`'s console password is `embsim`, for debugging a guest by hand
(`-serial stdio` on a copy). The guest has no route to anything but the host.

## Rebuilding

`build.sh` is idempotent: it reuses the downloaded base image (verified
against Debian's `SHA512SUMS`) and overwrites the output. Change `user-data`
and rebuild; the image carries no state from a previous build.
