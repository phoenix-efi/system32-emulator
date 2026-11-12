#!/bin/bash
# The run report prints the table writes (link type 5/7) delivered to each ECU and its acknowledgements, and flags any
# write that crossed the cable but was never answered. The always-on part uses generated toy images (a blinker ECU
# with no link stack: a delivered write is never answered, so the report must flag it). With S32_FIRMWARE_DIR (the
# stock images) paced writes are acknowledged and counted, and a refused write is reported as refused. SKIP without
# it, like the other firmware-driven checks here.
set -uo pipefail
R=$(CDPATH='' cd "$(dirname "$0")/../../.." && pwd)
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
fail=0
check() { if [ "$2" = "$3" ]; then echo "  ok   pair $1: $4"; else echo "  FAIL pair $1: $4 (got '$2', want '$3')"; fail=1; fi; }

# ---- always: generated images (scripts/conformance.py synthetic_firmware; no firmware ships with this repository) --
IMGS=$(python3 - "$R" "$T" <<'PY'
import importlib.util, pathlib, sys
repo, out = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
sys.path.insert(0, str(repo / "scripts"))
spec = importlib.util.spec_from_file_location("conformance", repo / "scripts" / "conformance.py")
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
fw = m.synthetic_firmware(out / "synth")
print(" ".join(f"{k}={v}" for k, v in fw.items()))
PY
) || { echo "  FAIL pair table-write-report: could not generate the toy images"; exit 1; }
CT=$(echo "$IMGS" | tr ' ' '\n' | sed -n 's/^ct=//p'); BFL=$(echo "$IMGS" | tr ' ' '\n' | sed -n 's/^bfl=//p')
PFL=$(echo "$IMGS" | tr ' ' '\n' | sed -n 's/^pfl=//p')
run() {   # run <name> <seconds> [VAR=value ...] -> $T/<name>.out (stdout + stderr)
  local n=$1 secs=$2; shift 2
  env S32_CYL=4 S32_PPM="$T/$n.ppm" "$@" "$R/emu/pair/build/s32pair_shim" "$CT" "$BFL" "$PFL" "$secs" > "$T/$n.out" 2>&1
}
run quiet 3
run write 3 S32_ECU_LINK="1:c2e:fuel-row:8=110:full@1.0"
check no_writes_no_flag "$(grep -c 'crossed the cable but w[a-z]* not acknowledged' "$T/quiet.out")" "0" "no table writes: nothing flagged"
check write_counted "$(grep -c 'table writes delivered to this ECU: fuel 1, ignition 0; acknowledged by it: fuel 0, ignition 0' "$T/write.out")" "1" \
      "the delivered write and the missing ACK are printed"
check write_flagged "$(grep -c '1 table write crossed the cable but was not acknowledged' "$T/write.out")" "1" \
      "the never-acknowledged write is flagged"
check write_delivered "$(grep -c 'delivered over the cable: CT->ECU 1 frames (bad 0)' "$T/write.out")" "1" \
      "the frame crossed the cable intact (receiving-end analyser)"

# ---- with the stock images ---------------------------------------------------------------------
if [ -z "${S32_FIRMWARE_DIR:-}" ]; then
  echo "  SKIP pair table-write-report (stock images): S32_FIRMWARE_DIR not set"
  [ "${PAIR_TEST_STRICT:-}" = 1 ] && { echo "  FAIL: skipped, and PAIR_TEST_STRICT=1 makes a skip a failure"; exit 1; }
else
  FW="$S32_FIRMWARE_DIR"
  pick() { { ls "$FW"/*"$1" 2>/dev/null || true; } | sort | head -n 1; }
  VCT=$(pick _prog.bin); VBFL=$(pick _bflash.bin); VPFL=$(pick _pflash.bin); FRAM="$FW/default_ecu.fram"
  if [ -z "$VCT" ] || [ -z "$VBFL" ] || [ -z "$VPFL" ] || [ ! -f "$FRAM" ]; then
    echo "  FAIL pair table-write-report: $FW needs *_prog.bin, *_bflash.bin, *_pflash.bin and default_ecu.fram"; exit 1
  fi
  vrun() {   # vrun <name> <link knob> -> $T/<name>.out
    local n=$1; shift
    env S32_ECU1_FRAM="$FRAM" S32_ECU2_FRAM="$FRAM" S32_CYL=4 S32_RPM=2400 S32_START=0 S32_INPUT="4.0:enc:5" \
        S32_PPM="$T/$n.ppm" S32_ECU_LINK="$1" \
        "$R/emu/pair/build/s32pair_shim" "$VCT" "$VBFL" "$VPFL" 8 > "$T/$n.out" 2>&1
  }
  vrun paced "1:c2e:fuel-row:8=110@6.47,2:c2e:fuel-row:8=110@6.47"
  S32_LINK_TABLE_SEAL=full vrun refused "1:c2e:fuel-row:8=110@6.2"
  check v86_paced_clean "$(grep -c 'crossed the cable but w[a-z]* not acknowledged' "$T/paced.out")" "0" \
        "stock images, paced writes: acknowledged by both ECUs, nothing flagged"
  check v86_paced_counted "$(grep -c 'acknowledged by it: fuel 1, ignition 0' "$T/paced.out")" "2" \
        "stock images, paced writes: the ACKs are printed"
  check v86_refused_said "$(grep -c '1 of those acknowledgements refused the write' "$T/refused.out")" "1" \
        "stock images, a write with the full-table seal: the ECU answers with a refusal, and the report says so"
  check v86_refused_not_lost "$(grep -c 'crossed the cable but w[a-z]* not acknowledged' "$T/refused.out")" "0" \
        "stock images, a refused write is answered, not flagged as never acknowledged"
fi
exit $fail
