#!/bin/bash
# 3-MCU co-simulator smoke test (Cockpit Controller + ECU1 + ECU2 + engine plant).
# Runs 6.0 s at 2400 RPM on firmware YOU supply and checks link frames, 30.0 deg advance (retard input LOW: documented
# harness), a 5.270 ms median injector pulse width (within 0.003 ms) and the parallel-union skew (values observed on the
# stock v86 images with default_ecu.fram).
# No firmware ships with this repository: set S32_FIRMWARE_DIR to a directory holding *_prog.bin and default_ecu.fram,
# plus full ECU images or application-only *_app.flat. Without it this test reports SKIP (exit 0); it never PASSes.
set -euo pipefail
R=$(CDPATH='' cd "$(dirname "$0")/../../.." && pwd)

if [ -z "${S32_FIRMWARE_DIR:-}" ]; then
  echo "  SKIP pair smoke: S32_FIRMWARE_DIR not set (no firmware ships with this repository)"
  [ "${PAIR_TEST_STRICT:-}" = 1 ] && { echo "  FAIL: skipped, and PAIR_TEST_STRICT=1 makes a skip a failure"; exit 1; }
  exit 0
fi
FW="$S32_FIRMWARE_DIR"
pick() { { ls "$FW"/*"$1" 2>/dev/null || true; } | sort | head -n 1; }
CT=$(pick _prog.bin); BFL=$(pick _bflash.bin); PFL=$(pick _pflash.bin); APP=$(pick _app.flat); FRAM="$FW/default_ecu.fram"
if [ -n "$APP" ] && { [ -z "$BFL" ] || [ -z "$PFL" ]; }; then BFL=none; PFL=$APP; fi
if [ -z "$CT" ] || [ -z "$BFL" ] || [ -z "$PFL" ] || [ ! -f "$FRAM" ]; then
  echo "  FAIL pair smoke: $FW needs *_prog.bin, default_ecu.fram and full ECU images or *_app.flat"
  exit 1
fi

out=$(S32_ECU1_FRAM="$FRAM" S32_ECU2_FRAM="$FRAM" \
      S32_RPM=2400 S32_START=2 S32_STATS_FROM=3 \
      "$R/emu/pair/build/s32pair_shim" "$CT" "$BFL" "$PFL" 6 2>/dev/null)

fail=0
check() {
  if [ "$2" = "$3" ]; then
    echo "  ok   pair $1 = $2"
  else
    echo "  FAIL pair $1 = '$2' (want '$3')"
    fail=1
  fi
}

check "ECU1 CT->ECU frames" "$(echo "$out" | awk '/^ECU1: CT->ECU/ {print ($3 >= 20 ? "ok" : $3)}')" "ok"
check "ECU2 CT->ECU frames" "$(echo "$out" | awk '/^ECU2: CT->ECU/ {print ($3 >= 20 ? "ok" : $3)}')" "ok"
check "ECU1 OC1 advance" "$(echo "$out" | awk '/coil OC1:/ {print $9; exit}')" "30.0"
check "ECU2 OC1 advance" "$(echo "$out" | awk '/coil OC1:/ {v=$9} END {print v}')" "30.0"
# within 0.003 ms of 5.270 (see the header): prints the value when it is outside, "ok" when inside
near() { awk -v v="$1" 'BEGIN { if (v != "" && v + 0 >= 5.267 && v + 0 <= 5.273) print "ok"; else print v }'; }
check "ECU1 median inj ms (5.270 +/- 0.003)" "$(near "$(echo "$out" | awk '/injector pulses/ {print $5; exit}')")" "ok"
check "ECU2 median inj ms (5.270 +/- 0.003)" "$(near "$(echo "$out" | awk '/injector pulses/ {v=$5} END {print v}')")" "ok"
check "parallel union skew" "$(echo "$out" | awk '/two live ECUs/ {print $18}')" "(+0.0%);"

exit $fail
