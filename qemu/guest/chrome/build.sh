#!/usr/bin/env bash
# Build the Chrome guest image for embsim-qemu.
#
# Downloads Debian's generic cloud image, boots it once under the host's
# accelerator with the cloud-init seed in user-data (served over HTTP on the
# host; the guest reaches the host at 10.0.2.2), waits for it to provision
# itself and power off, and flattens the result into one standalone qcow2.
#
#   guest/chrome/build.sh                    # host architecture, default cache
#   ARCH=x86_64 OUT=/tmp/chrome.qcow2 guest/chrome/build.sh
#
# Environment: ARCH (aarch64|x86_64, default: host), OUT (image path),
# EMBSIM_QEMU_CACHE (default ~/.cache/embsim/qemu), QEMU / QEMU_IMG
# (binaries), FIRMWARE (aarch64 UEFI image), ACCEL (hvf|kvm|tcg).
# Needs: qemu-system-<arch>, qemu-img, curl, python3, sha512sum or shasum.
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ARCH=${ARCH:-$(uname -m)}
[ "$ARCH" = arm64 ] && ARCH=aarch64
case $ARCH in
  aarch64) DEB=arm64;  KPKG=arm64; MACHINE=virt; CPU_TCG=cortex-a72 ;;
  x86_64)  DEB=amd64;  KPKG=amd64; MACHINE=q35;  CPU_TCG=max ;;
  *) echo "unsupported ARCH=$ARCH" >&2; exit 2 ;;
esac
CACHE=${EMBSIM_QEMU_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/embsim/qemu}
OUT=${OUT:-$CACHE/chrome-debian13-$ARCH.qcow2}
QEMU=${QEMU:-qemu-system-$ARCH}
QEMU_IMG=${QEMU_IMG:-qemu-img}
RELEASE=trixie
BASE_NAME=debian-13-genericcloud-$DEB.qcow2
BASE_URL=https://cloud.debian.org/images/cloud/$RELEASE/latest
BASE=$CACHE/$BASE_NAME
BUILD_TIMEOUT=${BUILD_TIMEOUT:-2400}   # seconds; apt over user-mode NAT is slow

log() { printf '[build-chrome] %s\n' "$*" >&2; }

command -v "$QEMU" >/dev/null || { log "missing $QEMU"; exit 2; }
command -v "$QEMU_IMG" >/dev/null || { log "missing $QEMU_IMG"; exit 2; }
mkdir -p "$CACHE"

# --- accelerator ---
if [ -z "${ACCEL:-}" ]; then
  case $(uname -s) in
    Darwin) ACCEL=hvf ;;
    Linux)  if [ -w /dev/kvm ]; then ACCEL=kvm; else ACCEL=tcg; fi ;;
    *)      ACCEL=tcg ;;
  esac
fi
case $ACCEL in
  hvf|kvm) ACCEL_ARGS=(-accel "$ACCEL" -cpu host) ;;
  tcg)     ACCEL_ARGS=(-accel tcg -cpu "$CPU_TCG"); log "TCG: expect a slow build" ;;
esac

# --- firmware (aarch64 boots UEFI) ---
FW_ARGS=()
if [ "$ARCH" = aarch64 ]; then
  if [ -z "${FIRMWARE:-}" ]; then
    PREFIX=$(cd "$(dirname "$(command -v "$QEMU")")/.." && pwd)
    for f in "$PREFIX/share/qemu/edk2-aarch64-code.fd" /opt/homebrew/share/qemu/edk2-aarch64-code.fd \
             /usr/local/share/qemu/edk2-aarch64-code.fd /usr/share/qemu/edk2-aarch64-code.fd /usr/share/AAVMF/AAVMF_CODE.fd; do
      [ -f "$f" ] && FIRMWARE=$f && break
    done
  fi
  [ -n "${FIRMWARE:-}" ] || { log "no aarch64 UEFI firmware found; set FIRMWARE"; exit 2; }
  FW_ARGS=(-bios "$FIRMWARE")
fi

# --- base image, verified against Debian's SHA512SUMS ---
if [ ! -f "$BASE" ]; then
  log "downloading $BASE_NAME"
  curl -fsSL -C - --retry 5 --retry-all-errors -o "$BASE.part" "$BASE_URL/$BASE_NAME"
  mv "$BASE.part" "$BASE"
fi
log "verifying $BASE_NAME"
EXPECTED=$(curl -fsSL "$BASE_URL/SHA512SUMS" | awk -v n="$BASE_NAME" '$2 == n {print $1}')
if command -v sha512sum >/dev/null; then ACTUAL=$(sha512sum "$BASE" | awk '{print $1}'); else ACTUAL=$(shasum -a 512 "$BASE" | awk '{print $1}'); fi
if [ -z "$EXPECTED" ]; then
  log "warning: $BASE_NAME not listed in SHA512SUMS (the 'latest' set moved?); continuing unverified"
elif [ "$EXPECTED" != "$ACTUAL" ]; then
  log "checksum mismatch for $BASE (delete it and rerun)"; exit 1
fi

# --- workspace: overlay disk, seed, console log ---
WORK=$(mktemp -d "${TMPDIR:-/tmp}/embsim-chrome-build.XXXXXX")
SEED_PID=""; QEMU_PID=""
cleanup() {
  [ -n "$QEMU_PID" ] && kill "$QEMU_PID" 2>/dev/null || true
  [ -n "$SEED_PID" ] && { kill "$SEED_PID" 2>/dev/null; wait "$SEED_PID" 2>/dev/null; } || true
  rm -rf "$WORK"
}
trap cleanup EXIT
"$QEMU_IMG" create -q -f qcow2 -F qcow2 -b "$BASE" "$WORK/build.qcow2" 8G
mkdir -p "$WORK/seed"
sed "s/__KERNEL_ARCH__/$KPKG/g" "$HERE/user-data" > "$WORK/seed/user-data"
printf 'instance-id: embsim-chrome-%s\nlocal-hostname: embsim-chrome\n' "$(date +%s)" > "$WORK/seed/meta-data"

# The seed server: any free loopback port; the guest sees the host as 10.0.2.2.
SEED_PORT=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1])')
( cd "$WORK/seed" && exec python3 -m http.server "$SEED_PORT" --bind 127.0.0.1 > "$WORK/seed.log" 2>&1 ) &
SEED_PID=$!

log "provisioning under $ACCEL (console: $WORK/console.log)"
"$QEMU" -M "$MACHINE" "${ACCEL_ARGS[@]}" -m 2G -smp 4 "${FW_ARGS[@]}" \
  -drive "file=$WORK/build.qcow2,if=virtio,format=qcow2" \
  -netdev user,id=n0 -device virtio-net-pci,netdev=n0 \
  -smbios "type=1,serial=ds=nocloud;s=http://10.0.2.2:$SEED_PORT/" \
  -display none -monitor none -serial "file:$WORK/console.log" \
  > "$WORK/qemu.log" 2>&1 &
QEMU_PID=$!

# cloud-init powers the guest off when it is done (power_state in user-data);
# a guest that is still up at the deadline did not get there.
START=$(date +%s)
while kill -0 "$QEMU_PID" 2>/dev/null; do
  if [ $(( $(date +%s) - START )) -ge "$BUILD_TIMEOUT" ]; then
    log "timed out after ${BUILD_TIMEOUT}s; console tail:"; tail -20 "$WORK/console.log" >&2; exit 1
  fi
  sleep 5
done
wait "$QEMU_PID" || true
QEMU_PID=""
if ! grep -q EMBSIM-PROVISIONED "$WORK/console.log"; then
  log "the guest powered off without provisioning; console tail:"; tail -40 "$WORK/console.log" >&2; exit 1
fi

# --- flatten into one standalone image ---
log "flattening into $OUT"
mkdir -p "$(dirname "$OUT")"
"$QEMU_IMG" convert -q -O qcow2 "$WORK/build.qcow2" "$OUT.tmp"
mv "$OUT.tmp" "$OUT"
log "built $OUT ($(du -h "$OUT" | cut -f1)) in $(( $(date +%s) - START ))s"
echo "$OUT"
