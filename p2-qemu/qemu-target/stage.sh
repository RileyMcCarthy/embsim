#!/bin/sh
# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (c) 2026 Riley McCarthy
#
# Stage the P2 target into a QEMU source tree (README.md, "Building").
#
# usage: stage.sh <qemu-src>
#        stage.sh --identity     print the target's identity and stage nothing
#
# Re-runnable. target/p2 and hw/p2 are replaced outright, because a quoted
# #include searches the includer's own directory first: a stale hand-made
# trans_stub.c.inc left there would shadow the one the build generates. A
# patch that is already applied is skipped.
#
# It also writes target/p2/hostipc-identity.h: the identity of the sources it
# staged, which the built qemu-system-p2 reports in its handshake and embsim
# checks against the copy it carries (embsim-p2-qemu's src/target.rs computes
# the same digest): the first 16 hex digits of the SHA-256 of a listing of
# every file here but README.md, the LICENSE-* files and dotfiles, one line
# each, sorted by path, as `sha256sum` prints them ("<digest>  <path>").
set -eu

if [ "$#" -ne 1 ]; then
    echo "usage: $0 <qemu-src> | --identity" >&2
    exit 2
fi
q=$1
here=$(cd "$(dirname "$0")" && pwd)

sha256() {
    if command -v sha256sum >/dev/null 2>&1; then
        sha256sum | cut -d' ' -f1
    else
        shasum -a 256 | cut -d' ' -f1
    fi
}

identity=$(
    cd "$here" &&
    find . -type f ! -name README.md ! -name 'LICENSE-*' ! -name '.*' |
        sed 's|^\./||' | LC_ALL=C sort |
        while IFS= read -r f; do
            printf '%s  %s\n' "$(sha256 < "$f")" "$f"
        done | sha256 | cut -c1-16
)
if [ "$q" = "--identity" ]; then
    echo "$identity"
    exit 0
fi
if [ ! -f "$q/meson.build" ] || [ ! -d "$q/target" ]; then
    echo "$q is not a QEMU source tree" >&2
    exit 1
fi

rm -rf "$q/target/p2" "$q/hw/p2"
mkdir -p "$q/target/p2" "$q/hw/p2" "$q/configs/devices/p2-softmmu"
cp "$here"/target-p2/* "$q/target/p2/"
cp "$here"/hw-p2/* "$q/hw/p2/"
cp "$here/p2-softmmu.mak" "$q/configs/targets/p2-softmmu.mak"
cp "$here/p2-softmmu-devices.mak" "$q/configs/devices/p2-softmmu/default.mak"
# The device set the linked node once built with, gone with it: a tree staged
# before carries a copy configure would still offer.
rm -f "$q/configs/devices/p2-softmmu/node.mak"
cat > "$q/target/p2/hostipc-identity.h" <<IDENTITY
/* Written by stage.sh: the identity of the target sources staged here. */
#define P2IPC_TARGET_IDENTITY "$identity"
IDENTITY
echo "stage.sh: target identity $identity"

for patch in register-p2.patch host-thread.patch; do
    if git -C "$q" apply --reverse --check "$here/$patch" 2>/dev/null; then
        echo "stage.sh: $patch is already applied"
    else
        git -C "$q" apply "$here/$patch"
        echo "stage.sh: applied $patch"
    fi
done
