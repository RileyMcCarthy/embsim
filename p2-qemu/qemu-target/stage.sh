#!/bin/sh
# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (c) 2026 Riley McCarthy
#
# Stage the P2 target into a QEMU source tree (README.md, "Building").
#
# usage: stage.sh <qemu-src>
#
# Re-runnable. target/p2 and hw/p2 are replaced outright, because a quoted
# #include searches the includer's own directory first: a stale hand-made
# trans_stub.c.inc left there would shadow the one the build generates. A
# patch that is already applied is skipped.
set -eu

if [ "$#" -ne 1 ]; then
    echo "usage: $0 <qemu-src>" >&2
    exit 2
fi
q=$1
here=$(cd "$(dirname "$0")" && pwd)

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
cp "$here/p2-softmmu-node-devices.mak" "$q/configs/devices/p2-softmmu/node.mak"

for patch in register-p2.patch host-thread.patch; do
    if git -C "$q" apply --reverse --check "$here/$patch" 2>/dev/null; then
        echo "stage.sh: $patch is already applied"
    else
        git -C "$q" apply "$here/$patch"
        echo "stage.sh: applied $patch"
    fi
done
