#!/bin/bash
# Snapshots (emu/common/s32snap.h): a run restored from a snapshot taken at t gives the same trace, the same logs and
# the same end-of-run report as running straight through, with the same knobs and with different knobs from t + 10 ms
# on; a run whose start or earlier stimulus differs is refused (exit 5). Generated images (scripts/conformance.py
# synthetic_firmware), no firmware needed; firmware cases are checked by scripts/conformance.py --builds ...,snapshot.
set -euo pipefail
R=$(CDPATH='' cd "$(dirname "$0")/../../.." && pwd)
P="$R/emu/pair/build/s32pair_shim"
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
python3 -I -c "import sys; sys.path.insert(0, '$R/scripts'); import conformance as C, pathlib; C.synthetic_firmware(pathlib.Path('$T/fw'))"
FW="$T/fw/synth_prog.bin $T/fw/synth_bflash.bin $T/fw/synth_pflash.bin"
run() {   # run <name> <secs> [VAR=value ...]
  local n=$1 s=$2; shift 2
  mkdir -p "$T/$n"
  ( cd "$T/$n" && env S32_RPM=1800 S32_START=0.1 S32_CYL=6 S32_AN=5=900 S32_TPS_NOISE=40 \
      S32_TRACE=trace.txt S32_SPARKLOG=spark.txt S32_INJLOG=inj.txt S32_STATUSLOG=status.txt S32_LINKLOG=link.txt \
      S32_LINKRAW=linkraw.txt S32_CT_CONSOLE=ct_console.txt S32_PPM=screen.ppm "$@" "$P" $FW "$s" >out.txt 2>err.txt ) || echo $? >"$T/$n/rc"
}
same() {  # same <a> <b>: trace, logs and report identical
  local f
  for f in trace.txt spark.txt inj.txt status.txt link.txt linkraw.txt ct_console.txt screen.ppm; do
    cmp -s "$T/$1/$f" "$T/$2/$f" || { echo "  FAIL pair snapshot: $f differs ($1 vs $2)"; return 1; }
  done
  diff -q <(grep -v ' wall \|^screen ->' "$T/$1/out.txt") <(grep -v ' wall \|^screen ->' "$T/$2/out.txt") >/dev/null ||
    { echo "  FAIL pair snapshot: end-of-run report differs ($1 vs $2)"; return 1; }
}
FUT=(S32_ECU_POWER=2:off@0.6,2:on@0.75 S32_CRANK_GLITCH=1:0.7 S32_AN_FAULT=1:7=raw:475@0.65 "S32_INPUT=0.62:menu,0.8:enc:1@20")
run straight 1.0
run base 1.0 S32_SNAPSHOT_OUT="$T/s04@0.4,$T/s05@0.55"
run again 1.0 S32_SNAPSHOT_IN="$T/s04"
run again2 1.0 S32_SNAPSHOT_IN="$T/s05"
run fstraight 1.0 "${FUT[@]}"
run frestored 1.0 "${FUT[@]}" S32_SNAPSHOT_IN="$T/s04"
run chained 1.0 "${FUT[@]}" S32_SNAPSHOT_IN="$T/s04" S32_SNAPSHOT_OUT="$T/s07@0.7"
run fromchain 1.0 "${FUT[@]}" S32_SNAPSHOT_IN="$T/s07"
run early 1.0 S32_ECU_POWER=2:off@0.405 S32_SNAPSHOT_IN="$T/s04"
run static 1.0 S32_AN=5=800 S32_SNAPSHOT_IN="$T/s04"
fail=0
for r in straight base again again2 fstraight frestored chained fromchain; do
  [ -f "$T/$r/rc" ] && { echo "  FAIL pair snapshot: run $r exit $(cat "$T/$r/rc")"; sed 's/^/      /' "$T/$r/err.txt" | tail -3; fail=1; }
done
[ -s "$T/straight/trace.txt" ] && [ -s "$T/straight/inj.txt" ] || { echo "  FAIL pair snapshot: straight run wrote no trace or injector log"; fail=1; }
same straight base || fail=1            # writing snapshots changes nothing
same straight again || fail=1           # restore at 0.4 s, same knobs
same straight again2 || fail=1          # restore at 0.55 s
same fstraight frestored || fail=1      # restore at 0.4 s, other faults and bezel input from 0.6 s on
same fstraight chained || fail=1        # a restored run writing a snapshot
same fstraight fromchain || fail=1      # restored from that snapshot
for r in early static; do
  if [ "$(cat "$T/$r/rc" 2>/dev/null)" != 5 ]; then echo "  FAIL pair snapshot: run $r was not refused (exit 5)"; fail=1; fi
done
[ $fail = 0 ] && echo "  ok   pair snapshot: restored runs (same knobs, other faults and input later, chained) identical to straight runs; mismatched starts refused"
exit $fail
