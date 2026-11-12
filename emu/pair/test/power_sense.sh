#!/bin/bash
# ECU power loss vs. a dead link, as the Controller sees them (emu/pair).
#
# The Controller senses each ECU's power on that ECU's cat5 cable (RA2 = ECU1, RA1 = ECU2, active high). A real power
# loss drops the pin; a powered ECU whose link goes silent (broken data pair, failed transceiver, hung ECU) does
# not. The two must stay separately testable:
#   S32_ECU_POWER=1:off@6   power loss  -> ECU1 tab GRAY ("not seen"); the trim readout shows ECU2's trim; the trim
#                                          knob now reaches ECU2 (type-9 frames to ECU2, ECU2's status trim moves)
#   S32_ECU_LINK=1:dead@6   dead link   -> ECU1 tab RED (the colour the procedures give a grounded P-lead)
#   S32_SWITCHES="plead on" P-lead      -> ECU1 tab RED, for comparison
# Everything is observed from outside: the Controller's framebuffer (tab colours, the trim readout) and the link
# bytes (emu/common/link_decode.c semantics, decoded here from S32_LINKRAW). Screen positions and colours are those of
# the stock firmware's home page, OBSERVED; like the smoke test's numbers they describe that firmware here.
# Firmware is bring-your-own (S32_FIRMWARE_DIR); without it this test reports SKIP.
set -euo pipefail
R=$(CDPATH='' cd "$(dirname "$0")/../../.." && pwd)
if [ -z "${S32_FIRMWARE_DIR:-}" ]; then
  echo "  SKIP pair power-sense: S32_FIRMWARE_DIR not set (no firmware ships with this repository)"
  [ "${PAIR_TEST_STRICT:-}" = 1 ] && { echo "  FAIL: skipped, and PAIR_TEST_STRICT=1 makes a skip a failure"; exit 1; }
  exit 0
fi
FW="$S32_FIRMWARE_DIR"
pick() { { ls "$FW"/*"$1" 2>/dev/null || true; } | sort | head -n 1; }
CT=$(pick _prog.bin); BFL=$(pick _bflash.bin); PFL=$(pick _pflash.bin); APP=$(pick _app.flat); FRAM="$FW/default_ecu.fram"
if [ -n "$APP" ] && { [ -z "$BFL" ] || [ -z "$PFL" ]; }; then BFL=none; PFL=$APP; fi
if [ -z "$CT" ] || [ -z "$BFL" ] || [ -z "$PFL" ] || [ ! -f "$FRAM" ]; then
  echo "  FAIL pair power-sense: $FW needs *_prog.bin, default_ecu.fram and full ECU images or *_app.flat"
  exit 1
fi
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
run() {   # run <name> <seconds> [VAR=value ...]
  local n=$1 secs=$2; shift 2
  env S32_ECU1_FRAM="$FRAM" S32_ECU2_FRAM="$FRAM" S32_RPM=2400 S32_START=2 S32_PPM="$T/$n.ppm" S32_LINKRAW="$T/$n.raw" \
      "$@" "$R/emu/pair/build/s32pair_shim" "$CT" "$BFL" "$PFL" "$secs" > "$T/$n.out" 2>&1
}
# Trim +3 at t = 4 s (three single edges; a NEGATIVE edge raises the trim), before any fault.
TRIM3="4:edge:-3@200"
run ref   9 S32_INPUT="$TRIM3,8.5:snap:$T/ref_snap"
run power 13 S32_ECU_POWER="1:off@6" S32_INPUT="$TRIM3,8.5:snap:$T/power_snap,9:edge:-2@200"
run link  10 S32_ECU_LINK="1:dead@6" S32_INPUT="$TRIM3"
run plead 8 S32_SWITCHES="plead on"

exec python3 - "$T" <<'PY'
import struct, sys
T = sys.argv[1]
fail = 0

def check(name, got, want):
    global fail
    ok = got == want
    print(f"  {'ok  ' if ok else 'FAIL'} pair {name} = {got}" + ("" if ok else f" (want {want})"))
    fail |= not ok

def ppm(path):
    d = open(path, "rb").read()
    i = d.index(b"255\n") + 4
    return d[i:]

def colour(px, x0, x1, y0, y1):
    """Dominant label colour in a box: blue, green, red, gray, or none."""
    n = {}
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            r, g, b = px[(y * 320 + x) * 3:(y * 320 + x) * 3 + 3]
            if (r, g, b) == (0, 0, 0) or min(r, g, b) > 240:
                continue
            c = ("red" if r > 200 and g < 80 and b < 80 else "blue" if b > 200 and r < 80 and g < 80 else
                 "green" if g > 150 and r < 80 and b < 80 else
                 "gray" if abs(r - g) < 24 and abs(g - b) < 24 and 90 < r < 210 else "other")
            n[c] = n.get(c, 0) + 1
    return max(n, key=n.get) if n else "none"

ECU1_TAB = (29, 68, 14, 25)        # "ECU1" tab label on the home page (OBSERVED)
TRIM_BOX = (170, 240, 175, 200)    # the Fuel Trim value

def box(px, b):
    x0, x1, y0, y1 = b
    return b"".join(px[(y * 320 + x0) * 3:(y * 320 + x1 + 1) * 3] for y in range(y0, y1 + 1))

def frames(path, direction):
    """Good frames (type, payload, t) on one direction of the wire, deframed as emu/common/link_decode.h does."""
    out, cur, esc = [], None, False
    for line in open(path):
        p = line.split()
        if len(p) < 3 or p[1] != direction:
            continue
        t = float(p[0])
        for b in (int(x, 16) for x in p[2:]):
            if b == 0x7E:
                if cur is not None and len(cur) >= 4 and crc(cur) == 0:
                    out.append((cur[0] | cur[1] << 8, bytes(cur[2:-2]), t))
                cur, esc = bytearray(), False
            elif cur is None:
                continue
            elif b == 0x7D:
                esc = True
            else:
                cur.append(b ^ 0x20 if esc else b)
                esc = False
    return out

def crc(b):
    c = 0xFFFF
    for x in b:
        c ^= x
        for _ in range(8):
            c = (c >> 1) ^ 0x8408 if c & 1 else c >> 1
    return c

def last_trim(path, direction):
    st = [f for f in frames(path, direction) if f[0] == 2 and len(f[1]) == 48]
    return struct.unpack_from("<i", st[-1][1], 20)[0] if st else None

# power loss
px = ppm(f"{T}/power.ppm")
check("ECU1 power loss: ECU1 tab", colour(px, *ECU1_TAB), "gray")
check("ECU1 power loss: trim readout shows ECU2's trim (+3, as the healthy run)",
      box(ppm(f"{T}/power_snap.ppm"), TRIM_BOX) == box(ppm(f"{T}/ref_snap.ppm"), TRIM_BOX), True)
n9 = len([f for f in frames(f"{T}/power.raw", "CT->ECU2") if f[0] == 9 and f[2] >= 9.0])
check("ECU1 power loss: knob sends trim deltas to ECU2", n9 >= 2, True)
check("ECU1 power loss: ECU2 trim after +2 more", last_trim(f"{T}/power.raw", "ECU2->CT"), 5)
stale = box(px, TRIM_BOX) == box(ppm(f"{T}/power_snap.ppm"), TRIM_BOX)
print(f"  info pair ECU1 power loss: trim readout after the knob moved ECU2 to +5: "
      f"{'still shows +3 (does not follow)' if stale else 'changed'}")
# dead link, P-lead
check("ECU1 link dead (powered): ECU1 tab", colour(ppm(f"{T}/link.ppm"), *ECU1_TAB), "red")
check("ECU1 P-lead grounded: ECU1 tab", colour(ppm(f"{T}/plead.ppm"), *ECU1_TAB), "red")
sys.exit(1 if fail else 0)
PY
