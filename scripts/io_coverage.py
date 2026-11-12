#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""I/O coverage gate: everything the firmware touches must be in the I/O ledger, and everything the ledger calls
modelled must have a test.

    python3 -I scripts/io_coverage.py [--firmware-dir DIR] [--keep DIR] [--json OUT]      (make io-coverage)

Runs the firmware you supply (S32_FIRMWARE_DIR or --firmware-dir: *_prog.bin, *_bflash.bin, *_pflash.bin,
default_ecu.fram; optional ecu1_maps.fram / ecu2_maps.fram / ct.fram, configuration chips of your own for a six-cylinder
engine) in emu/pair with the I/O
coverage recorder on (S32_IOCOV), over a scripted run set: cold boot on blank FRAM, engine start/run/stop with every
cockpit switch moved, the bezel menus, SD map load and datalog, firmware-update entry, the ECU SD slot, and every fault
knob. The union of what each MCU touched -- SFR words read or written, digital input pins sampled, output pins
configured, ADC channels converted -- is compared with model/io_ledger.json:

  FAIL  an SFR whose peripheral module no ledger entry claims;
  FAIL  an input pin, output pin or ADC channel that no ledger entry lists;
  FAIL  a ledger entry marked modelled whose test does not exist;
  FAIL  a knob the ledger names that the emulator does not have (s32pair --knobs).

Without firmware it prints SKIP and exits 0 (the static checks -- tests and knobs exist -- still run and can fail).
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

REPO = pathlib.Path(__file__).resolve().parent.parent
PAIR = REPO / "emu" / "pair" / "build" / "s32pair_shim"
LEDGER = REPO / "model" / "io_ledger.json"
MCU_IO = REPO / "model" / "mcu_io.json"
PORTS = "ABCDEFG"

# SFR name -> peripheral module (the unit the ledger's "sfr_modules" lists). Order matters: first match wins.
MODULE_RULES = [
    (r"^(TRIS|PORT|LAT|ODC)([A-G])$", lambda m: "PORT" + m.group(2)),
    (r"^ANS([A-G])$", lambda m: "PORT" + m.group(1)),
    (r"^(CNPUE|CNEN|CNCON|CNPU\d|CNPD\d|CNEN\d)", lambda m: "CN"),
    (r"^(AD1|ADC1BUF)", lambda m: "ADC1"),
    (r"^U(\d)(MODE|STA|TXREG|RXREG|BRG)", lambda m: "UART" + m.group(1)),
    (r"^SPI(\d)(CON\d?|STAT|BUF|BRG)", lambda m: "SPI" + m.group(1)),
    (r"^(T|TMR|PR)(\d)(CON)?$", lambda m: "TMR" + m.group(2)),
    (r"^OC(\d)(CON\d?|R|RS)$", lambda m: "OC" + m.group(1)),
    (r"^IC(\d)", lambda m: "IC" + m.group(1)),
    (r"^(INTCON\d?|INTSTAT|IPTMR|IFS\d|IEC\d|IPC\d+|INTTREG)$", lambda m: "INT"),
    (r"^(NVM)", lambda m: "NVM"),
    (r"^(OSCCON|OSCTUN|CLKDIV\d?|REFOCON|SYSKEY|OSCCONL|OSCCONH)", lambda m: "OSC"),
    (r"^(RPINR\d+|RPOR\d+)$", lambda m: "PPS"),
    (r"^G1", lambda m: "GFX"),
    (r"^(BMX|CHE|DMACON|DDPCON|DCH)", lambda m: "SYSBUS"),
    (r"^(WREG\d+|SPLIM|PCL|PCH|DSRPAG|DSWPAG|RCOUNT|SR|CORCON|DISICNT|TBLPAG)$", lambda m: "CPU"),
    (r"^(RCON|WDTCON|RSWRST)$", lambda m: "RESET"),
    (r"^PMD\d", lambda m: "PMD"),
]


def module_of(name: str) -> str:
    for rx, fn in MODULE_RULES:
        m = re.match(rx, name)
        if m:
            return fn(m)
    return "UNKNOWN:" + name


def sfr_names() -> dict[str, dict[int, str]]:
    p32 = {int(s["addr"], 16): s["name"] for s in json.loads((REPO / "model" / "pic32mx_sfr.json").read_text())["sfrs"]
           if "PIC32MX795F512L" in s["parts"]}
    p24 = {s["addr"]: s["name"] for s in json.loads((REPO / "model" / "pic24fj256da210_sfr.json").read_text())["sfrs"]}
    return {"ecu": p32, "controller": p24}


def find_firmware(d: pathlib.Path) -> dict[str, str] | None:
    out = {}
    for key, pat in (("ct", "*_prog.bin"), ("bfl", "*_bflash.bin"), ("pfl", "*_pflash.bin")):
        c = sorted(d.glob(pat))
        if not c:
            return None
        out[key] = str(c[0])
    if not (d / "default_ecu.fram").is_file():
        return None
    out["fram"] = str(d / "default_ecu.fram")
    for k in ("ecu1_maps.fram", "ecu2_maps.fram", "ct.fram"):
        if (d / k).is_file():
            out[k] = str(d / k)
    upd = sorted(p for p in d.iterdir() if re.fullmatch(r"EC\d{6}\.bin", p.name, re.I))
    if upd:                                                  # the ECU update file is optional
        out["update"] = str(upd[0])
    return out


def sdcard(path: pathlib.Path, files: dict[str, bytes]) -> bool:
    """A FAT32 card image with `files`; False if pyfatfs (scripts/pyrun, uv.lock) is unavailable."""
    tool = [str(REPO / "scripts" / "pyrun"), "-W", "ignore", "-I", str(REPO / "scripts" / "sdcard_image.py")]
    try:
        if subprocess.run(tool + ["new", str(path), "64"], capture_output=True).returncode:
            return False
        for name, data in files.items():
            src = path.parent / ("_" + name)
            src.write_bytes(data)
            if subprocess.run(tool + ["put", str(path), str(src), "/" + name], capture_output=True).returncode:
                return False
    except FileNotFoundError:
        return False
    return True


def fuel_csv() -> bytes:
    """A 14 x 9 fuel map in the Controller's map-file format (comment line, MAP axis row, one row per RPM)."""
    maps = [1, 6, 10, 14, 18, 21, 24, 27, 30]
    rpms = [50, 600, 800, 1000, 1300, 1700, 2100, 2200, 2300, 2400, 2500, 2600, 2700, 3000]
    lines = ["# FUEL MAP", "MAP," + ",".join(map(str, maps))]
    for i, r in enumerate(rpms):
        lines.append(f"IGN{r}," + ",".join(str(20 + 3 * j + i) for j in range(len(maps))))
    return ("\r\n".join(lines) + "\r\n").encode()


def run_set(fw: dict[str, str], tmp: pathlib.Path) -> list[tuple[str, dict[str, str], float]]:
    e1 = fw.get("ecu1_maps.fram", fw["fram"])
    e2 = fw.get("ecu2_maps.fram", fw["fram"])
    cyl = "6" if "ecu1_maps.fram" in fw else "4"
    maps = {"S32_ECU1_FRAM": e1, "S32_ECU2_FRAM": e2, "S32_CYL": cyl, "S32_MAG2": "77" if cyl == "6" else "80"}
    if "ct.fram" in fw:
        maps["S32_CT_FRAM"] = fw["ct.fram"]
    runs: list[tuple[str, dict[str, str], float]] = []
    runs.append(("cold boot, blank FRAM (factory defaults written)", {}, 5.0))
    ct_card = tmp / "ct_card.img"
    have_card = sdcard(ct_card, {"FUEL.CSV": fuel_csv()})
    eng = dict(maps, S32_RPM_AT="2=150,3=900,4.5=2400,8=1500,10=0", S32_AN="5=800,6=600,7=500,9=500",
               S32_TPS_NOISE="8",
               S32_SWITCH="5:1:plead=stop,5.4:1:plead=run,6:select=2,7:select=1,6.5:1:retard=on,7.5:1:retard=off,"
                          "6:1:dip1=on,6.2:2:dip4=on,6:pump2=on,6.5:pump1=off,7:digin3=lo,7.2:digin4=lo,8:2:inj=gnd")
    if have_card:
        eng["S32_CT_SD"] = str(tmp / "card_engine.img")
        shutil.copy(ct_card, eng["S32_CT_SD"])
    runs.append(("engine start / run / stop, every cockpit switch moved, datalog", eng, 12.0))
    menus = dict(maps, S32_INPUT="3:monitor,4:monitor,5:monitor,6:monitor,7:tune,8:edge:-1,9:knob,10:tune,11:menu,"
                                 "12:menu,13:menu,14:edge:-2,15:knob,16:edge:-1,17:knob,18:tune,18.5:monitor,19:menu,20:menu,"
                                 "21:menu,22:tune+monitor:1600,24:edge:-2,25:knob,26:tune,27:menu,28:menu,29:menu,30:edge:-3,"
                                 "31:knob,32:edge:-1,33:knob,34:enc:2,35:knob,36:menu,37:menu,38:menu,39:edge:-4,40:knob,41:menu,"
                                 "42:menu,43:enc:3,44:enc:-3")
    runs.append(("bezel: every menu, configs, advanced configs, date/time, tuning, firmware screen", menus, 45.0))
    if have_card:
        sd = dict(maps, S32_CT_SD=str(tmp / "card_maps.img"), S32_INPUT="5:menu,6:edge:-1,7:knob,10:knob,11:menu,"
                                                                      "12:knob,15:knob,16:menu,17:edge:-4,18:knob,19:knob")
        shutil.copy(ct_card, sd["S32_CT_SD"])
        runs.append(("SD: load FUEL and IGN maps (previous maps written back), firmware-update entry", sd, 22.0))
        ecu_sd = dict(maps, S32_ECU1_SD=str(tmp / "ecu1.img"), S32_ECU2_SD=str(tmp / "ecu2.img"),
                      S32_DEV_FAULT="1:sd=remove@3,1:sd=insert@3.7,1:sd=remove@4.5,1:sd=insert@5.2,2:sd=readerr@2-4,"
                                    "ct:sd=writeerr@6-7", S32_ECU_SD_WP="1")
        shutil.copy(ct_card, ecu_sd["S32_ECU1_SD"])
        shutil.copy(ct_card, ecu_sd["S32_ECU2_SD"])
        ecu_sd["S32_CT_SD"] = str(tmp / "card_ecusd.img")
        shutil.copy(ct_card, ecu_sd["S32_CT_SD"])
        runs.append(("ECU SD slots: card at boot, removal / insertion, read and write errors", ecu_sd, 8.0))
    if have_card:
        cfg = tmp / "cfg_card.img"
        if sdcard(cfg, {"CFG.CSV": b"CONFIG2,6\r\nCONFIG7,2750\r\n"}):
            runs.append(("ECU SD: configuration file loaded at boot over an unreadable configuration",
                         {"S32_ECU1_SD": str(cfg), "S32_ECU2_FRAM": fw["fram"]}, 5.0))
        if "update" in fw:
            name = pathlib.Path(fw["update"]).name.upper()
            other = name[:7] + ("1" if name[7] == "0" else "0") + ".BIN"
            upd = tmp / "upd_card.img"
            if sdcard(upd, {other: pathlib.Path(fw["update"]).read_bytes()}):
                runs.append(("ECU SD: update file of another version (boot loader erases and reprograms)",
                             {"S32_ECU1_SD": str(upd), "S32_ECU1_FRAM": fw["fram"], "S32_ECU2_FRAM": fw["fram"]}, 12.0))
    faults = dict(maps, S32_RPM="2400", S32_START="2", S32_CRANK_JITTER="20", S32_CRANK_GLITCH="1:5.0,2:5.5:20:sync",
                  S32_CRANK_DROP="2:6-6.05:sync,1:6.2-6.25", S32_CRANK_EMI="1:500@7-7.5", S32_HALL_STUCK="2:crank=low@8-8.2",
                  S32_AN_FAULT="1:5=gnd@5-5.1,1:6=open@5.2-5.3,2:7=vcc@5.4,1:9=float@6-6.5,1:8=raw:300@3,1:10=noise:20@3,"
                               "1:12=raw:600@3,1:13=raw:100@3,1:14=raw:900@3,ct:13=raw:400@3,ct:15=intermittent:50@4-5,ct:12=stuck@6",
                  S32_PIN_FAULT="1:RB2=intermittent:100@9-9.5,ct:RG1=chatter:20@4-6",
                  S32_ECU_LINK="1:ber=0.001@6-6.5,2:garbage=50@7,2:cut-e2c@8-8.5",
                  S32_SUPPLY="2:5.5@9,2:13.8@9.3", S32_RESET="1@10,ct@10.5", S32_RTC="osf", S32_BOUNCE_MS="5",
                  S32_DEV_FAULT="1:fram=absent@10.5-10.6,ct:rtc=miso-low@11-11.1", S32_XTAL_PPM="1=50,2=-50,ct=30",
                  S32_CONSOLE_IN="1:1:0.3:get CONFIG2\\r;2:4:6:get CONFIG2\\r;ct:1:3:status\\r;ct:2:3:status\\r")
    runs.append(("faults: crank, analogue, wires, links, brown-out, MCLR, devices, RTC, consoles", faults, 12.0))
    runs.append(("power: ECU2 never powered, ECU1 power cycle, both off (Controller off)",
                 dict(maps, S32_RPM="1800", S32_ECU_POWER="2:off@0,1:off@4,1:on@5,2:on@6,1:off@8,2:off@8,1:on@9"), 11.0))
    return runs


def check_static(ledger: dict, knobs: set[str]) -> list[str]:
    errs = []
    for e in ledger["signals"] + [dict(c, modelled="yes") for c in ledger.get("on_chip", [])]:
        if e.get("modelled") in ("yes", "partial"):
            for t in e.get("tests", []):
                f = REPO / t.split("::")[0]
                if not f.exists():
                    errs.append(f"{e['id']}: test {t} does not exist")
                elif "::" in t and t.split("::")[1] not in f.read_text():
                    errs.append(f"{e['id']}: test {t}: '{t.split('::')[1]}' not found in {f.name}")
            if not e.get("tests"):
                errs.append(f"{e['id']}: marked modelled but no test")
        for k in e.get("knobs", []):
            name = k.split("=")[0].split(" ")[0]
            if name.startswith("S32_") and name not in knobs:
                errs.append(f"{e['id']}: knob {name} is not an emulator knob")
    return errs


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--firmware-dir", type=pathlib.Path, default=None)
    ap.add_argument("--keep", type=pathlib.Path, default=None, help="keep run outputs here")
    ap.add_argument("--json", type=pathlib.Path, default=None, help="write the merged coverage here")
    a = ap.parse_args()
    if a.firmware_dir is None and os.environ.get("S32_FIRMWARE_DIR"):
        a.firmware_dir = pathlib.Path(os.environ["S32_FIRMWARE_DIR"])
    ledger = json.loads(LEDGER.read_text())
    if not PAIR.exists():
        print(f"FAIL: {PAIR} not built (make -C emu/pair)")
        return 1
    knobs = {ln.split()[0] for ln in subprocess.run([str(PAIR), "--knobs"], capture_output=True, text=True).stdout.splitlines() if ln.strip()}
    errs = check_static(ledger, knobs)
    for e in errs:
        print("  FAIL ledger", e)
    if a.firmware_dir is None:
        print("  SKIP io-coverage run set: no firmware (set S32_FIRMWARE_DIR or --firmware-dir; none ships with this repo)")
        return 1 if errs else 0
    fw = find_firmware(a.firmware_dir.resolve())
    if fw is None:
        print(f"  FAIL io-coverage: {a.firmware_dir} needs *_prog.bin, *_bflash.bin, *_pflash.bin, default_ecu.fram")
        return 1
    tmp = pathlib.Path(tempfile.mkdtemp(prefix="iocov-")) if a.keep is None else a.keep
    tmp.mkdir(parents=True, exist_ok=True)
    union: dict[str, dict] = {}
    for i, (label, env, secs) in enumerate(run_set(fw, tmp)):
        cov = tmp / f"run{i}.json"
        full = dict(os.environ, S32_IOCOV=str(cov), S32_PPM=str(tmp / f"run{i}.ppm"), **env)
        r = subprocess.run([str(PAIR), fw["ct"], fw["bfl"], fw["pfl"], str(secs)], env=full, capture_output=True, text=True, cwd=tmp)
        (tmp / f"run{i}.out").write_text(r.stdout + r.stderr)
        if r.returncode or not cov.exists():
            print(f"  FAIL run {i} ({label}): exit {r.returncode}")
            errs.append(f"run {i} failed")
            continue
        print(f"  ran  {i}: {label} ({secs:.0f} s)")
        d = json.loads(cov.read_text())
        for mcu in ("controller", "ecu1", "ecu2"):
            u = union.setdefault(mcu, {"rd": set(), "wr": set(), "in": [0] * 7, "out": [0] * 7, "an": set()})
            u["rd"] |= set(d[mcu]["sfr_read"])
            u["wr"] |= set(d[mcu]["sfr_write"])
            for i_p, p in enumerate(PORTS):
                u["in"][i_p] |= int(d[mcu]["pin_in"][p], 16)
                u["out"][i_p] |= int(d[mcu]["pin_out"][p], 16)
            u["an"] |= {c for c, _ in d[mcu]["an_conv"]}
    names = sfr_names()
    io = json.loads(MCU_IO.read_text())["parts"]
    impl = {"ecu": io["PIC32MX795F512L"]["ports"], "controller": io["PIC24FJ256DA210"]["ports"]}
    claimed: dict[str, dict[str, set]] = {"ecu": {"pins": set(), "adc": set(), "mod": set()},
                                          "controller": {"pins": set(), "adc": set(), "mod": set()}}
    for e in ledger["signals"] + ledger.get("on_chip", []):
        for mcu in (["ecu", "controller"] if e["board"] == "both" else [e["board"]]):
            claimed[mcu]["pins"] |= set(e.get("mcu_pins", []))
            claimed[mcu]["adc"] |= set(e.get("adc", []))
            claimed[mcu]["mod"] |= set(e.get("sfr_modules", []))
    report = {}
    for mcu in ("controller", "ecu1", "ecu2"):
        kind = "controller" if mcu == "controller" else "ecu"
        u = union[mcu]
        sfrs = sorted({names[kind].get(int(x, 16), x) for x in u["rd"] | u["wr"]})
        mods = sorted({module_of(n) for n in sfrs})
        pins_in = sorted(f"R{p}{b}" for i_p, p in enumerate(PORTS) for b in range(16)
                         if u["in"][i_p] >> b & 1 and b in impl[kind].get(p, []))
        pins_out = sorted(f"R{p}{b}" for i_p, p in enumerate(PORTS) for b in range(16)
                          if u["out"][i_p] >> b & 1 and b in impl[kind].get(p, []))
        adc = sorted(f"AN{c}" for c in u["an"])
        miss_mod = [m for m in mods if m not in claimed[kind]["mod"]]
        miss_pin = [p for p in sorted(set(pins_in) | set(pins_out)) if p not in claimed[kind]["pins"]]
        miss_adc = [c for c in adc if c not in claimed[kind]["adc"]]
        report[mcu] = {"sfrs": sfrs, "modules": mods, "pins_in": pins_in, "pins_out": pins_out, "adc": adc,
                       "unledgered": {"modules": miss_mod, "pins": miss_pin, "adc": miss_adc}}
        print(f"  {mcu}: {len(sfrs)} SFRs in {len(mods)} modules, {len(pins_in)} input pins, {len(pins_out)} output pins, "
              f"{len(adc)} ADC channels touched")
        for what, miss in (("module", miss_mod), ("pin", miss_pin), ("ADC channel", miss_adc)):
            for m in miss:
                print(f"  FAIL {mcu}: {what} {m} touched by the firmware but not in model/io_ledger.json")
                errs.append(f"{mcu} {what} {m}")
    if a.json:
        a.json.write_text(json.dumps(report, indent=1) + "\n")
    if a.keep is None:
        shutil.rmtree(tmp, ignore_errors=True)
    if errs:
        print(f"FAIL: io-coverage: {len(errs)} problem(s)")
        return 1
    print("PASS: io-coverage: everything the firmware touched is in the ledger, and every modelled entry has a test")
    return 0


if __name__ == "__main__":
    sys.exit(main())
