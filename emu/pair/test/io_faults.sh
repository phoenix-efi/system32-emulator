#!/bin/bash
# Every external signal family of the target system, driven through the harness knobs (emu/common/s32board.c) on the
# firmware YOU supply, with its effect observed only from outside: driver pins (coil / injector pulses), the harness
# observers (injectors at the engine, tach), the link bytes (frames, status frames), the consoles, the SD / FRAM / RTC
# chips and the Controller's power state. Each check is named; model/io_ledger.json cites them as io_faults.sh::<name>.
# Numbers are what the stock firmware does with default_ecu.fram (4 cylinders); they describe that firmware in this
# emulator, they are not a specification. No firmware ships with this repository: without S32_FIRMWARE_DIR every check
# reports SKIP (exit 0).
set -uo pipefail
R=$(CDPATH='' cd "$(dirname "$0")/../../.." && pwd)
if [ -z "${S32_FIRMWARE_DIR:-}" ]; then
  echo "  SKIP pair io-faults: S32_FIRMWARE_DIR not set (no firmware ships with this repository)"
  [ "${PAIR_TEST_STRICT:-}" = 1 ] && { echo "  FAIL: skipped, and PAIR_TEST_STRICT=1 makes a skip a failure"; exit 1; }
  exit 0
fi
FW="$S32_FIRMWARE_DIR"
pick() { { ls "$FW"/*"$1" 2>/dev/null || true; } | sort | head -n 1; }
CT=$(pick _prog.bin); BFL=$(pick _bflash.bin); PFL=$(pick _pflash.bin); APP=$(pick _app.flat); FRAM="$FW/default_ecu.fram"
if [ -n "$APP" ] && { [ -z "$BFL" ] || [ -z "$PFL" ]; }; then BFL=none; PFL=$APP; fi
if [ -z "$CT" ] || [ -z "$BFL" ] || [ -z "$PFL" ] || [ ! -f "$FRAM" ]; then
  echo "  FAIL pair io-faults: $FW needs *_prog.bin, default_ecu.fram and full ECU images or *_app.flat"; exit 1
fi
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
run() {   # run <name> <seconds> [VAR=value ...] -> $T/<name>.out (stdout + stderr)
  local n=$1 secs=$2; shift 2
  env S32_ECU1_FRAM="$FRAM" S32_ECU2_FRAM="$FRAM" S32_CYL=4 S32_PPM="$T/$n.ppm" "$@" \
      "$R/emu/pair/build/s32pair_shim" "$CT" "$BFL" "$PFL" "$secs" > "$T/$n.out" 2>&1
}
fail=0
check() { if [ "$2" = "$3" ]; then echo "  ok   io $1: $4"; else echo "  FAIL io $1: $4 (got '$2', want '$3')"; fail=1; fi; }
blk() { awk -v e="ECU$2 INT1" '$0 ~ "^"e {s=1; next} /^ECU[12] INT1/ {s=0} /^harness:/ {s=0} s' "$T/$1.out"; }   # ECU k's output block
coils() { blk "$1" "$2" | grep -c "coil OC[0-9]: "; }
adv() { blk "$1" "$2" | awk '/coil OC1:/ {print $9; exit}'; }
advmax() { blk "$1" "$2" | awk '/coil OC/ {gsub(/\)/, "", $15); if ($15 + 0 > m) m = $15 + 0} END {print m + 0}'; }
ownmax() { blk "$1" "$2" | awk '/own cylinder:/ {for (i = 1; i <= NF; i++) if ($i == "max") {v = $(i + 1); gsub(/[^0-9.]/, "", v); if (v + 0 > m) m = v + 0}} END {print m + 0}'; }
med() { blk "$1" "$2" | awk '/injector pulses/ {print $5; exit}'; }
enr() { blk "$1" "$2" | awk '/injector pulses/ {match($0, /\(([0-9.]+)%\)/); print substr($0, RSTART + 1, RLENGTH - 3) + 0; f=1} END {if (!f) print -1}'; }
inj1() { awk '/^injector 1 \(at the engine/ {match($0, /: [0-9]+ openings/); print substr($0, RSTART + 2, RLENGTH - 11) + 0; f=1} END {if (!f) print 0}' "$T/$1.out"; }
frames() { awk -v e="ECU$2:" '$1 == e {s=1} s && /delivered over the cable/ {print $6, $11, $14; exit}' "$T/$1.out" | tr -d ',();'; }   # at the receivers: CT->ECU frames, ECU->CT frames, ECU->CT bad
gt() { awk -v a="$1" -v b="$2" 'BEGIN {print (a + 0 > b + 0) ? "yes" : "no"}'; }

# ---- switches and the ECU Select harness
run base 7 S32_RPM=2400 S32_START=2 S32_STATS_FROM=3
run plead 7 S32_RPM=2400 S32_START=2 S32_STATS_FROM=3 S32_SWITCH="2.5:1:plead=stop"
check plead_stop_kills_spark "$(coils plead 1)/$(coils plead 2)" "0/$(coils base 2)" "ECU1 P-lead grounded: no ECU1 coil pulses, ECU2 unchanged"
run sel1 8 S32_RPM=2400 S32_START=2 S32_ECU_POWER="2:off@4"
run sel2 8 S32_RPM=2400 S32_START=2 S32_ECU_POWER="2:off@4" S32_ECU_SELECT=2
check select2_moves_fuel_to_ecu2 "$(gt "$(inj1 sel1)" "$(( $(inj1 sel2) + 50 ))")" "yes" \
      "ECU Select on ECU2 then ECU2 lost: injector 1 opened $(inj1 sel2) times vs $(inj1 sel1) with ECU1 selected"
check select2_harness_line "$(grep -c 'ECU Select ECU2; injector enable wires ECU1 grounded ECU2 open' "$T/sel2.out")" "1" "harness reports the grounded enable wire"
run ret 7 S32_RPM=2400 S32_START=2 S32_STATS_FROM=3 S32_RETARD=on
check retard_on_subtracts_config "$(awk -v a="$(adv base 1)" -v b="$(adv ret 1)" 'BEGIN {print (a - b > 0.5) ? "yes" : "no"}')" "yes" \
      "retard input HIGH: advance $(adv ret 1) deg vs $(adv base 1) deg"

# ---- analogue sensors
run map 7 S32_RPM=2400 S32_START=2 S32_STATS_FROM=4 S32_AN_FAULT="1:5=gnd@3"
check map_short_gnd_shortens_pulse "$(gt "$(med map 2)" "$(med map 1)")" "yes" "ECU1 MAP shorted to ground: pulse $(med map 1) ms vs ECU2 $(med map 2) ms"
run iat 7 S32_RPM=2400 S32_START=2 S32_STATS_FROM=4 S32_AN_FAULT="1:6=gnd@3"
check iat_fault_changes_pulse "$([ "$(med iat 1)" != "$(med iat 2)" ] && echo yes || echo no)" "yes" "ECU1 IAT faulted: pulse $(med iat 1) ms vs ECU2 $(med iat 2) ms"
check et_ecu2_open_by_wiring "$(awk '/^ECU2: CT->ECU/ {s=1} s && /last status frame/ {match($0, /ET [0-9]+ F/); print substr($0, RSTART + 3, RLENGTH - 5); exit}' "$T/base.out")" "9000" \
      "ECU2 reports ET 9000 (open input): the ET sensor is wired to ECU1 only"
run tps3 6 S32_RPM=1800 S32_START=2 S32_STATS_FROM=3 S32_TPS_FAULT="1:500~4@2,2:500~3@2"
check tps_noise_enriches_low_rpm "$(gt "$(enr tps3 1)" 5)/$(enr tps3 2)" "yes/0" "TPS noise +-4 counts at 1800 RPM: ECU1 $(enr tps3 1)% of pulses enriched, +-3: ECU2 $(enr tps3 2)%"
run tpsf 8 S32_RPM=1800 S32_START=2 S32_STATS_FROM=4 S32_ECU_POWER="1:off@3" S32_TPS_UNPOWERED=float
run tpsg 8 S32_RPM=1800 S32_START=2 S32_STATS_FROM=4 S32_ECU_POWER="1:off@3"
check tps_floating_after_ecu1_loss "$(gt "$(enr tpsf 2)" 10)/$(enr tpsg 2)" "yes/0" "ECU1 (TPS supply) lost: floating wiper $(enr tpsf 2)% enriched on ECU2, grounded wiper $(enr tpsg 2)%"

# ---- crank and sync sensors
run emi 7 S32_RPM=2400 S32_START=2 S32_STATS_FROM=3 S32_CRANK_EMI="1:200@4-6"
check crank_glitch_early_spark "$(gt "$(advmax emi 1)" "$(echo "$(advmax emi 2) + 10" | bc)")" "yes" "extra crank pulses on ECU1: earliest spark $(advmax emi 1) deg BTDC vs ECU2 $(advmax emi 2)"
run sync 8 S32_RPM=2400 S32_START=2 S32_STATS_FROM=5 S32_CRANK_DROP="1:4-99:sync" S32_CRANK_GLITCH="1:4.2528"
check sync_drop_lost_sync "$(gt "$(ownmax sync 1)" 60)/$(gt "$(ownmax sync 2)" 60)" "yes/no" "sync lost + one extra pulse: ECU1 fires up to $(ownmax sync 1) deg before its own cylinder's TDC, ECU2 $(ownmax sync 2)"
check tach_pulses_per_rev "$(awk '/^ECU1 outputs: tach/ {gsub(/\(/, "", $6); print $6; exit}' "$T/base.out")" "80.0" "tach out at 2400 RPM, 4 cylinders: 2 pulses per revolution"

# ---- links, consoles
run ber 7 S32_RPM=1200 S32_START=2 S32_ECU_LINK="1:ber=0.002@3"
check link_ber_bad_frames "$(gt "$(frames ber 1 | awk '{print $3}')" 0)" "yes" "bit errors on ECU1's pair: $(frames ber 1 | awk '{print $3}') bad frames at the Controller"
run cut 7 S32_RPM=1200 S32_START=2 S32_ECU_LINK="2:cut-e2c@3"
check link_cut_one_direction "$(gt "$(frames base 2 | awk '{print $2}')" "$(( $(frames cut 2 | awk '{print $2}') + 3 ))")/$(gt "$(frames cut 2 | awk '{print $1}')" 10)" "yes/yes" \
      "ECU2->Controller pair cut: $(frames cut 2 | awk '{print $2}') frames reach the Controller, Controller->ECU2 still $(frames cut 2 | awk '{print $1}')"
run con 3 ECU_CONSOLE="$T/con" S32_CONSOLE_IN='1:1:0.4:get CONFIG2\r'
check console_ecu_get "$(sed 's/\x1b\[[0-9;-]*[A-Za-z]//g' "$T/con1.txt" | tr -s '\r\n' '\n' | grep -c '^CONFIG2=')" "1" "typed 'get CONFIG2' on ECU1's console: it answers"

# ---- storage devices
run fram 4 ECU_CONSOLE="$T/fc" S32_DEV_FAULT="2:fram=absent@0"
check fram_absent_defaults "$(sed 's/\x1b\[[0-9;-]*[A-Za-z]//g' "$T/fc2.txt" | grep -c 'Loading default CFG')/$(sed 's/\x1b\[[0-9;-]*[A-Za-z]//g' "$T/fc1.txt" | grep -c 'Loading default CFG')" "1/0" "ECU2's FRAM not answering: ECU2 reloads its defaults, ECU1 does not"
if command -v uv >/dev/null 2>&1 && "$R/scripts/pyrun" -W ignore -I "$R/scripts/sdcard_image.py" new "$T/card.img" 64 >/dev/null 2>&1; then
  printf '# FUEL MAP\r\nMAP,1,6,10,14,18,21,24,27,30\r\n' > "$T/FUEL.CSV"
  for r in 50 600 800 1000 1300 1700 2100 2200 2300 2400 2500 2600 2700 3000; do printf 'IGN%s,40,41,42,43,44,45,46,47,48\r\n' "$r" >> "$T/FUEL.CSV"; done
  "$R/scripts/pyrun" -W ignore -I "$R/scripts/sdcard_image.py" put "$T/card.img" "$T/FUEL.CSV" /FUEL.CSV >/dev/null 2>&1
  cp "$T/card.img" "$T/ecu.img"
  run sdl 12 S32_CT_SD="$T/card.img" S32_ECU1_FRAM_OUT="$T/e1.fram" S32_INPUT="5:menu,6:edge:-1,7:knob"
  check ct_sd_load_fuel "$(cmp -s "$FRAM" "$T/e1.fram" && echo same || echo changed)/$(gt "$(awk '/^Controller SD:/ {print $6}' "$T/sdl.out")" 0)" "changed/yes" \
        "Main Menu -> Load FUEL from the Controller's card: ECU1's FRAM map changed and the previous map was written to the card"
  run ecusd 6 S32_ECU1_SD="$T/ecu.img"
  check ecu_sd_card_mounts "$(gt "$(awk '/^ECU1 SD:/ {print $3}' "$T/ecusd.out")" 0)" "yes" "card in ECU1's slot: the ECU reads and writes it"
  "$R/scripts/pyrun" -W ignore -I "$R/scripts/sdcard_image.py" new "$T/cfg.img" 64 >/dev/null 2>&1
  printf 'CONFIG2,6\r\nCONFIG7,2750\r\n' > "$T/CFG.CSV"
  "$R/scripts/pyrun" -W ignore -I "$R/scripts/sdcard_image.py" put "$T/cfg.img" "$T/CFG.CSV" /CFG.CSV >/dev/null 2>&1
  run cfg 5 S32_ECU1_FRAM= S32_ECU1_SD="$T/cfg.img" S32_ECU1_FRAM_OUT="$T/cfg.fram"
  check ecu_sd_cfg_csv_loads "$(python3 -c "import sys; b=open(sys.argv[1],'rb').read(); print(b[0x1490:0x14A0].split(b'\\0')[0].decode(), b[0x14E0:0x14F0].split(b'\\0')[0].decode())" "$T/cfg.fram")" "6 2750" \
        "ECU1 boots with an unreadable configuration and CFG.CSV on its card: CONFIG2 / CONFIG7 come from the card"
  ec=$(ls "$FW"/EC[0-9][0-9][0-9][0-9][0-9][0-9].[bB][iI][nN] 2>/dev/null | head -n 1)
  if [ -n "$ec" ]; then                                   # the update file itself is optional in S32_FIRMWARE_DIR
    n=$(basename "$ec" | tr a-z A-Z); m="EC000000.BIN"
    "$R/scripts/pyrun" -W ignore -I "$R/scripts/sdcard_image.py" new "$T/u1.img" 64 >/dev/null 2>&1; cp "$T/u1.img" "$T/u2.img"
    "$R/scripts/pyrun" -W ignore -I "$R/scripts/sdcard_image.py" put "$T/u1.img" "$ec" "/$n" >/dev/null 2>&1
    "$R/scripts/pyrun" -W ignore -I "$R/scripts/sdcard_image.py" put "$T/u2.img" "$ec" "/${n:0:7}$([ "${n:7:1}" = 0 ] && echo 1 || echo 0).BIN" >/dev/null 2>&1
    run upd1 8 S32_ECU1_SD="$T/u1.img"
    run upd2 12 S32_ECU1_SD="$T/u2.img"
    check ecu_sd_update_file "$(awk '/^RTC chip:/ {print $15}' "$T/upd1.out")/$(gt "$(awk '/^RTC chip:/ {split($15, a, "/"); print a[1]}' "$T/upd2.out")" 0)" "0/0/yes" \
          "update file on ECU1's card: same version -> no reflash; a file whose name gives another version -> the boot loader erases program flash"
  else
    echo "  SKIP io ecu_sd_update_file: no EC*.bin update file in S32_FIRMWARE_DIR (optional)"
  fi
  run fw 12 S32_CT_SD="$T/ecu.img" S32_CT_PFLASH_OUT="$T/ct.pfl" S32_INPUT="5:menu,6:edge:-4,7:knob,8:knob"
  check ct_firmware_update_entry "$(gt "$(awk '/^RTC chip:/ {print $7}' "$T/fw.out")" 0)" "yes" "Firmware -> Update Firmware: the Controller erases and rewrites flash (update flag)"
  check ct_pflash_out "$(wc -c < "$T/ct.pfl" | tr -d ' ')" "262656" "S32_CT_PFLASH_OUT writes the 262656-byte Controller program flash image"
  check ct_firmware_update_resets "$(grep -c '^Controller illegal-opcode reset (RCON.IOPUWR)' "$T/fw.out")/$(awk '/^Controller resets since power-on:/ {print $5}' "$T/fw.out")/$(gt "$(awk '$1 == "t=12s" {print $3}' "$T/fw.out")" "$(awk '$1 == "t=9s" {print $3}' "$T/fw.out")")" "1/1/yes" \
        "Update Firmware with no loader in flash: the closing BREAK resets the Controller (illegal opcode, RCON.IOPUWR) and the program restarts and talks to the ECUs again"
else
  echo "  SKIP io ct_sd_load_fuel / ecu_sd_card_mounts / ecu_sd_cfg_csv_loads / ecu_sd_update_file / ct_firmware_update_entry / ct_firmware_update_resets: needs uv (scripts/pyrun, pyfatfs from uv.lock) to make a card image"
  if [ "${PAIR_TEST_STRICT:-}" = 1 ]; then echo "  FAIL io card cases: skipped, and PAIR_TEST_STRICT=1 makes a skip a failure (run inside nix develop)"; fail=1; fi
fi
run rtcinv 4 S32_RTC=invalid
run rtcosf 4 S32_RTC=osf
check rtc_osf_boot "$(awk '/^RTC chip:/ {print substr($3, 1, 4); exit}' "$T/rtcinv.out")/$(awk '/^RTC chip:/ {print substr($3, 1, 4); exit}' "$T/rtcosf.out")/$(grep -c 'oscillator-stop flag set' "$T/rtcosf.out")" "2019/2026/1" \
      "RTC registers invalid at power-on: the Controller sets 2019 (no message); oscillator-stop flag alone: ignored (flag still set)"

# ---- power and reset
run bor 8 S32_RPM=2400 S32_START=2 S32_STATS_FROM=6 S32_SUPPLY="1:5.0@4,1:13.8@4.2"
check brownout_reboots_ecu "$(grep -c 'ECU1 power ON (cold boot) at t=4.200' "$T/bor.out")/$(coils bor 1)" "1/$(coils base 1)" "ECU1 supply sags to 5 V for 0.2 s: held in reset, cold boot, sparks again"
run mclr 8 S32_RPM=2400 S32_START=2 S32_STATS_FROM=6 S32_RESET="1@4"
check mclr_reset_ecu "$(grep -c 'ECU1 MCLR reset at t=4.000' "$T/mclr.out")/$(coils mclr 1)" "1/$(coils base 1)" "MCLR pulse on ECU1: reset and back to sparking"
mkdir -p "$T/snaps"
run snap 1 S32_SNAPDIR="$T/snaps" S32_SNAP_MS=250
check panel_snapshot_period "$(cd "$T/snaps" && ls | tr '\n' ' ')" "m0000250.ppm m0000500.ppm m0000750.ppm m0001000.ppm " \
      "S32_SNAP_MS=250: the panel saved every 250 ms of a 1 s run, named by the millisecond"
run off 6 S32_ECU_POWER="1:off@3,2:off@3"
check both_ecus_off_controller_off "$(grep -c 'Controller power OFF at t=3.000' "$T/off.out")/$(grep -c 'Controller unpowered: dark' "$T/off.out")" "1/1" "both ECU breakers out: the Controller (powered by the cat5 cables) goes dark"
exit $fail
