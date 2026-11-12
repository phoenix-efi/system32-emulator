#!/bin/bash
# The text logs written from the MCU threads (S32_LINKLOG, S32_STATUSLOG, S32_SPARKLOG, S32_INJLOG) are the same bytes on
# every run and with the MCUs on one thread (S32_SERIAL=1): lines are buffered per thread and merged between quanta in
# emulated-time order. Firmware YOU supply (S32_FIRMWARE_DIR, with default_ecu.fram); SKIP without it.
set -euo pipefail
R=$(CDPATH='' cd "$(dirname "$0")/../../.." && pwd)
if [ -z "${S32_FIRMWARE_DIR:-}" ]; then echo "  SKIP pair log order: S32_FIRMWARE_DIR not set"; [ "${PAIR_TEST_STRICT:-}" = 1 ] && { echo "  FAIL: skipped, and PAIR_TEST_STRICT=1 makes a skip a failure"; exit 1; }; exit 0; fi
FW="$S32_FIRMWARE_DIR"
pick() { { ls "$FW"/*"$1" 2>/dev/null || true; } | sort | head -n 1; }
CT=$(pick _prog.bin); BFL=$(pick _bflash.bin); PFL=$(pick _pflash.bin); APP=$(pick _app.flat)
if [ -n "$APP" ] && { [ -z "$BFL" ] || [ -z "$PFL" ]; }; then BFL=none; PFL=$APP; fi
[ -f "$FW/default_ecu.fram" ] || { echo "  SKIP pair log order: $FW has no default_ecu.fram"; [ "${PAIR_TEST_STRICT:-}" = 1 ] && { echo "  FAIL: skipped, and PAIR_TEST_STRICT=1 makes a skip a failure"; exit 1; }; exit 0; }
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
run() {
  mkdir -p "$T/$1"
  cp "$FW/default_ecu.fram" "$T/$1.e1.fram"; cp "$FW/default_ecu.fram" "$T/$1.e2.fram"
  ( cd "$T/$1" && S32_SERIAL="$2" S32_ECU1_FRAM="$T/$1.e1.fram" S32_ECU2_FRAM="$T/$1.e2.fram" \
    S32_CYL=4 S32_RPM=2400 S32_START=2 S32_AN="5=900,9=600" \
    S32_LINKLOG=link.txt S32_STATUSLOG=status.txt S32_SPARKLOG=spark.txt S32_INJLOG=inj.txt S32_PPM=screen.ppm \
    "$R/emu/pair/build/s32pair_shim" "$CT" "$BFL" "$PFL" 8 >/dev/null 2>&1 )
}
run a 0; run b 0; run s 1
fail=0
for f in link.txt status.txt spark.txt inj.txt; do
  if [ ! -s "$T/a/$f" ]; then echo "  FAIL pair log order: $f is empty"; fail=1; continue; fi
  for r in b s; do
    if cmp -s "$T/a/$f" "$T/$r/$f"; then :; else echo "  FAIL pair log order: $f differs (threads run a vs $r)"; fail=1; fi
  done
done
[ $fail = 0 ] && echo "  ok   pair log order: link, status, spark and injector logs identical over 3 runs (threads x2, serial)"
exit $fail
