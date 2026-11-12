#!/bin/bash
# The Controller model must never run on a core library built from a different pic24.h.
#  1. Build dependency: after pic24.h changes (here: a field inserted before cfg), `make` in emu/controller rebuilds
#     libpic24.a and the objects that include pic24.h. The old rule had no prerequisites, so a library left from an
#     older header was kept and every pic24_cpu field after the change was read at the wrong offset.
#  2. Runtime guard: a library and objects from different headers stop in s32ct_init with a message, not run.
# Works on a scratch copy of emu/pic24, emu/controller and emu/common; the tree under test is not touched.
set -euo pipefail
HERE=$(CDPATH= cd -- "$(dirname "$0")/../.." && pwd)            # emu/
REPO=$(CDPATH= cd -- "$HERE/.." && pwd)
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
mkdir -p "$T/emu" "$T/model"
for d in pic24 controller common; do rsync -a --exclude build "$HERE/$d/" "$T/emu/$d/"; done
cp "$REPO/model/pic24fj256da210_sfr.json" "$T/model/"
M="make -s -C $T/emu/controller"
$M build/test_s32ct >/dev/null
"$T/emu/controller/build/test_s32ct" >/dev/null
cp "$T/emu/pic24/build/libpic24.a" "$T/old_libpic24.a"
mtime() { stat -c %Y "$1" 2>/dev/null || stat -f %m "$1"; }   # GNU first: GNU `stat -f` is file-system status
L0=$(mtime "$T/emu/pic24/build/libpic24.a"); O0=$(mtime "$T/emu/controller/build/s32ct.o")
sleep 1
# a layout change in the middle of pic24_cpu (before cfg), like the one that left a stale library behind
perl -0pi -e 's/\n  pic24_config cfg;\n\} pic24_cpu;/\n  uint32_t stale_probe[4];\n  pic24_config cfg;\n} pic24_cpu;/' "$T/emu/pic24/include/pic24.h"
grep -q stale_probe "$T/emu/pic24/include/pic24.h" || { echo "stale_core: FAIL (could not edit the scratch pic24.h)"; exit 1; }
$M build/test_s32ct >/dev/null
L1=$(mtime "$T/emu/pic24/build/libpic24.a"); O1=$(mtime "$T/emu/controller/build/s32ct.o")
[ "$L1" -gt "$L0" ] || { echo "stale_core: FAIL (libpic24.a not rebuilt after pic24.h changed)"; exit 1; }
[ "$O1" -gt "$O0" ] || { echo "stale_core: FAIL (s32ct.o not rebuilt after pic24.h changed)"; exit 1; }
"$T/emu/controller/build/test_s32ct" >/dev/null
# guard: objects from the new header linked against the library from the old one
cc -I"$T/emu/controller/include" -I"$T/emu/common" -I"$T/emu/pic24/include" -I"$T/emu/pic24/build" -I"$T/emu/pic24/tests" \
   -std=gnu11 -O2 "$T/emu/controller/tests/test_s32ct.c" "$T/emu/controller/build/libs32ct.a" "$T/old_libpic24.a" -lz \
   -o "$T/mixed" 2>/dev/null
set +e; bash -c "\"$T/mixed\" >\"$T/mixed.out\" 2>&1" 2>/dev/null; rc=$?; set -e
[ $rc -ne 0 ] && grep -q "different pic24.h" "$T/mixed.out" \
  || { echo "stale_core: FAIL (mixed build ran, rc=$rc)"; cat "$T/mixed.out" | head -5; exit 1; }
echo "stale_core: OK (core library rebuilt after a pic24.h layout change; a mixed build stops in s32ct_init)"
