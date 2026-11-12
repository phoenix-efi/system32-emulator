#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Stimulus cases for the emulator's own gates (scripts/conformance.py, scripts/bench.py): the case format, how a
case's inputs are prepared from a firmware directory, and the boot-once snapshot planning helpers.

A firmware directory holds the images the native front end takes (scripts/fetch_firmware.py writes one for the stock
v86 images):

    *_prog.bin                          Controller program memory (3 bytes per instruction word)
    *_bflash.bin + *_pflash.bin         ECU boot flash and program flash, or an application-only *_app.flat
    default_ecu.fram                    the ECU configuration chip as the firmware writes it on its first power-up

A case is JSON:

    {"name": "...", "about": "...", "boot_code": true (optional: needs the ECU boot region),
     "stimulus": {"preset": "default" | "blank",
                  "secs": 8 | "{H+3}",                      # run length; {H+x} = x s after the Controller's home page
                  "knobs": {"S32_RPM": "2400", ...},          # harness knobs (s32pair --knobs)
                  "fram": {"ecu1" | "ecu2" | "ct": "none" | "blank" | "<file in the firmware directory>" | "case:<name>"},
                  "cards": {"ct" | "ecu1" | "ecu2": {"files": {"NAME.CSV": {"text": "..."}}} | {"unformatted": true}}}}

Presets: "default" = default_ecu.fram on both ECUs, the Controller's chip blank; "blank" = every chip blank (the
firmware's first power-up). "case:<name>" takes the chip a case that ran earlier saved at exit.

A firmware project's own behavioural cases may use a richer case format with their own preparation; scripts/conformance.py
takes those through --provider (docs/conformance.md, "Case providers"). Standard library only, apart from the SD card
tool (scripts/sdcard_image.py through scripts/pyrun, pyfatfs from uv.lock) for cases with cards.

The snapshot planning helpers (stimulus, agree_until, snap_point, snap_spec, HomeTime) describe the emulator's
snapshot rules (emu/common/s32snap.h).
"""
from __future__ import annotations

import json
import math
import os
import pathlib
import re
import subprocess
import threading

REPO = pathlib.Path(__file__).resolve().parent.parent
PAIR = pathlib.Path(os.environ.get("CONFORMANCE_PAIR") or REPO / "emu" / "pair" / "build" / "s32pair_shim")
CASES = REPO / "tests" / "conformance" / "cases.json"
SCENARIOS = REPO / "tests" / "bench" / "scenarios.json"
FRAM_BYTES = 131072
HOME_RE = re.compile(r"\{H(?:([+-])([0-9]*\.?[0-9]+))?\}")
SNAP_MIN_GAIN = 0.5                                  # seconds of emulation a restore must save to be worth a snapshot
SNAP_QUANTUM = 1e-4                                  # emulator quantum (emu/pair QUANTUM_US)
HOME_GUESS_S = 4.0     # planning only: where "{H+x}" items fall among fixed-time ones; the emulator checks every restore
SNAP_GUARD = 0.010     # emulator S32SNAP_GUARD_S: items this close after a snapshot point must match too
NO_BOOT = pathlib.Path("none")      # the emulator's "no boot region" argument (application start)
SDTOOL = [str(REPO / "scripts" / "pyrun"), "-W", "ignore", "-I", str(REPO / "scripts" / "sdcard_image.py")]
_SDTOOL_LOCK = threading.Lock()
_SDTOOL_READY = False

PRESETS = {
    "default": {"ecu1": "default_ecu.fram", "ecu2": "default_ecu.fram", "ct": None},
    "blank": {"ecu1": None, "ecu2": None, "ct": None},
}


class Missing(Exception):
    """The firmware directory or the host lacks something this case needs: the case SKIPs with the message."""


def find_firmware(d: pathlib.Path) -> tuple[dict | None, str]:
    """The images of a firmware directory. The ECU is either a full image (*_bflash.bin + *_pflash.bin) or the
    application only (*_app.flat): with no boot region the emulator starts the application in the state the boot code
    hands over in (application start, emu/ecu/src/ecu.c), and fw["app_start"] is True."""
    if not d.is_dir():
        return None, f"{d} is not a directory"
    out: dict = {}
    c = sorted(d.glob("*_prog.bin"))
    if not c:
        return None, f"{d} has no *_prog.bin"
    out["ct"] = c[0]
    bfl, pfl, app = sorted(d.glob("*_bflash.bin")), sorted(d.glob("*_pflash.bin")), sorted(d.glob("*_app.flat"))
    if bfl and pfl:
        out["bfl"], out["pfl"], out["app_start"] = bfl[0], pfl[0], False
    elif app and not bfl:
        out["bfl"], out["pfl"], out["app_start"] = NO_BOOT, app[0], True
    else:
        return None, f"{d} has no ECU image (*_bflash.bin + *_pflash.bin, or an application-only *_app.flat)"
    if not (d / "default_ecu.fram").is_file():
        return None, f"{d} has no default_ecu.fram"
    out["dir"] = d
    return out, ""


# ---- home-page times --------------------------------------------------------------------------------------------

def home_time(text: str, h: float) -> str:
    """Replace every "{H}" / "{H+x}" in a string by the home-page time h plus the offset (seconds); refuse "{H-x}"."""
    def one(m: re.Match) -> str:
        if m.group(1) == "-":
            raise ValueError(f"pre-home anchor '{m.group(0)}' is refused")
        off = float(m.group(2)) if m.group(2) else 0.0
        return f"{h + off:.4f}".rstrip("0").rstrip(".")
    return HOME_RE.sub(one, text)


def home_sub(obj, h: float):
    """home_time over a JSON value; a string that becomes a plain number turns into a float."""
    if isinstance(obj, dict):
        return {k: home_sub(v, h) for k, v in obj.items()}
    if isinstance(obj, list):
        return [home_sub(v, h) for v in obj]
    if isinstance(obj, str) and HOME_RE.search(obj):
        t = home_time(obj, h)
        try:
            return float(t)
        except ValueError:
            return t
    return obj


# ---- the provider interface (scripts/conformance.py) -------------------------------------------------------------

def load_cases(tier: str, only: list[str] | None, home: float | None) -> list[dict]:
    """Every case: {"id", "group" (dict with "id"), "case" (dict), "deps" (case ids)}. The emulator's own conformance
    cases (tests/conformance/cases.json) and the benchmark scenarios (tests/bench/scenarios.json). tier: "full" adds
    the cases marked "tier": "full"."""
    out: list[dict] = []

    def want(cid: str) -> bool:
        return only is None or any(cid.startswith(o) for o in only)

    for prefix, path, key in (("emu", CASES, "cases"), ("bench", SCENARIOS, "scenarios")):
        for c in json.loads(path.read_text())[key]:
            cid = f"{prefix}/{c['name']}"
            if not want(cid) or (tier == "fast" and c.get("tier", "fast") != "fast" and not only):
                continue
            cc = json.loads(json.dumps(c))
            if home is not None:
                cc = home_sub(cc, home)
            deps = sorted({f"{prefix}/{v[5:]}" for v in cc.get("stimulus", {}).get("fram", {}).values()
                           if isinstance(v, str) and v.startswith("case:")})
            out.append({"id": cid, "group": {"id": prefix}, "case": cc, "deps": deps})
    return out


def needs_boot_code(group: dict, case: dict) -> str | None:
    """Why a case cannot run without the ECU boot region (application-only firmware), or None."""
    return "needs the ECU boot region" if case.get("boot_code") else None


def sdtool(args: list[str], **kw) -> subprocess.CompletedProcess:
    """Run the SD card image tool; the first call, under a lock, lets uv build the project environment once."""
    global _SDTOOL_READY
    with _SDTOOL_LOCK:
        if not _SDTOOL_READY:
            subprocess.run(SDTOOL[:1] + ["-c", "import pyfatfs"], capture_output=True, text=True)
            _SDTOOL_READY = True
    return subprocess.run(SDTOOL + args, **kw)


def prepare(group: dict, case: dict, fw: dict, d: pathlib.Path, done: dict[str, pathlib.Path]) -> tuple[dict, str]:
    """A case's inputs in d (FRAM and SD card images) and the environment of its emulator run: (env, secs). done maps
    an earlier case's name to its reference run directory (for "case:<name>" sources)."""
    st = case.get("stimulus", {})
    preset = PRESETS[st.get("preset", "default")]
    env: dict[str, str] = {}
    fr = st.get("fram", {})
    for chip, key, envk in (("ecu1", "e1", "S32_ECU1_FRAM"), ("ecu2", "e2", "S32_ECU2_FRAM"), ("ct", "ct", "S32_CT_FRAM")):
        src = fr.get(chip, preset[chip])
        if src is None or src == "none":
            continue
        if src == "blank":
            img = bytes(FRAM_BYTES)                          # all zero, as a new chip reads
        elif src.startswith("case:"):
            p = done.get(src[5:])
            if p is None or not (p / f"{key}.fram").exists():
                raise Missing(f"case {src[5:]} left no {key}.fram")
            img = (p / f"{key}.fram").read_bytes()
        else:
            p = fw["dir"] / src
            if not p.is_file():
                raise Missing(f"needs {src} in the firmware directory")
            img = p.read_bytes()
        (d / f"{key}_in.fram").write_bytes(img)
        env[envk] = str(d / f"{key}_in.fram")
        env[envk + "_OUT"] = str(d / f"{key}.fram")
    for chip, key in (("ecu1", "e1"), ("ecu2", "e2"), ("ct", "ct")):
        envk = {"ecu1": "S32_ECU1_FRAM", "ecu2": "S32_ECU2_FRAM", "ct": "S32_CT_FRAM"}[chip]
        if envk not in env:                                  # a blank chip still saves what the firmware wrote
            env[envk + "_OUT"] = str(d / f"{key}.fram")
    for slot, card in st.get("cards", {}).items():
        sdenv = {"ct": "S32_CT_SD", "ecu1": "S32_ECU1_SD", "ecu2": "S32_ECU2_SD"}[slot]
        img = d / f"card_{slot}.img"
        if card.get("unformatted"):
            with open(img, "wb") as f:
                f.truncate(64 * 1024 * 1024)
            env[sdenv] = str(img)
            continue
        files = d / f"cardsrc_{slot}"
        files.mkdir()
        for name, spec in card.get("files", {}).items():
            (files / name).write_bytes(spec["text"].encode())
        try:
            r = sdtool(["mk", str(img), str(files)], capture_output=True, text=True)
        except FileNotFoundError:
            raise Missing("needs uv (pyfatfs) to build SD card images")
        if r.returncode:
            raise Missing(f"SD card image tool failed: {r.stderr.strip()[-200:]}")
        env[sdenv] = str(img)
    env.update({k: str(v) for k, v in st.get("knobs", {}).items()})
    raw = st.get("secs", 6.0)
    secs = raw if isinstance(raw, str) else f"{float(raw):g}"
    env.update(S32_PPM=str(d / "screen.ppm"))
    return env, secs


# ---- boot-once snapshot planning -------------------------------------------------------------------------------------

def stimulus(env: dict, secs: float, fw: dict, d: pathlib.Path) -> dict | None:
    """What a case's knobs set up, as the emulator parsed them (S32_STIMULUS): the static part's hash and every timed
    item. None when the emulator cannot tell (an older binary, a failed start)."""
    out = d / "stimulus.json"
    r = subprocess.run([str(PAIR), str(fw["ct"]), str(fw["bfl"]), str(fw["pfl"]), str(secs)],
                       env=dict(os.environ, **env, S32_STIMULUS=str(out)), cwd=d, capture_output=True, text=True)
    if r.returncode or not out.exists():
        return None
    try:
        return json.loads(out.read_text())
    except ValueError:
        return None


class HomeTime(float):
    """A planning time that stands for "{H+x}", x = self - HOME_GUESS_S: the emulator places it at the home page (the
    runner does not know that moment; it plans with HOME_GUESS_S and the emulator refuses a restore that does not fit)."""
    def __eq__(self, o: object) -> bool:
        return isinstance(o, HomeTime) and float(self) == float(o)

    def __ne__(self, o: object) -> bool:
        return not self.__eq__(o)

    def __hash__(self) -> int:
        return hash(("H", float(self)))


def item_time(x: list) -> float:
    """An S32_STIMULUS item's time for planning: [kind, sub, t, hash] (seconds) or [kind, sub, x, hash, 1] (home page
    + x). An item before the home page ("{H-x}") is compared whatever the snapshot point, and so is one less than the
    guard after it ("{H}"): a snapshot taken before the home page cannot tell that it acts later than the guard (the
    emulator's check, emu/common/s32snap.h)."""
    if len(x) > 4 and x[4]:
        return -1.0 if x[2] < SNAP_GUARD else HomeTime(HOME_GUESS_S + round(x[2], 6))
    return float(x[2])


def plan_secs(secs) -> float:
    """A run length for planning: seconds, or "{H+x}" as a HomeTime."""
    if isinstance(secs, str) and HOME_RE.search(secs):
        return HomeTime(float(home_time(secs, HOME_GUESS_S)))
    return float(secs)


def snap_spec(t: float) -> str:
    """How S32_SNAPSHOT_OUT names a point: "4.5000", or "{H+0.9900}" for a HomeTime."""
    return f"{{H+{float(t) - HOME_GUESS_S:.4f}}}" if isinstance(t, HomeTime) else f"{t:.4f}"


def agree_until(a: list, b: list) -> float:
    """Time up to which two runs' timed items are the same: every item before it is equal, in the same order
    (math.inf when they never differ; a HomeTime when the first difference is a home-page item). Items are
    [kind, sub, time, hash(, home)]. Agreement up to t implies agreement up to any earlier time, so the first time they
    differ is found by bisection."""
    times = sorted({item_time(x) for x in a} | {item_time(x) for x in b})

    def same(t: float) -> bool:
        return [(x[0], x[1], x[3]) for x in a if item_time(x) <= t] == [(x[0], x[1], x[3]) for x in b if item_time(x) <= t]
    lo, hi = 0, len(times)
    while lo < hi:
        mid = (lo + hi) // 2
        if same(times[mid]):
            lo = mid + 1
        else:
            hi = mid
    return times[lo] if lo < len(times) else math.inf


def snap_point(agree: float, guard: float, secs_leader: float, secs_member: float) -> float:
    """The latest quantum boundary t with t + guard <= agree, no later than either run's end. A HomeTime when the limit
    is a home-page time; a home-page point less than guard + one quantum after the home page is not taken (0.0): its
    restore would be refused. When a home-page item lies exactly at point + guard the point is one quantum earlier:
    the emulator places home-page times through the SB_HOME_T0 sentinel (about 1e-10 s of rounding), so such an item
    could count as before the guard's end and the restore be refused."""
    lim = agree - guard
    if isinstance(agree, HomeTime):
        lim = HomeTime(lim)
    t = min((lim, plan_secs(secs_leader), plan_secs(secs_member)), key=float)
    if isinstance(t, HomeTime):
        k = (float(t) - HOME_GUESS_S) / SNAP_QUANTUM
        n = math.floor(k + 1e-6)
        if isinstance(agree, HomeTime) and float(t) == float(lim) and abs(k - round(k)) < 1e-6:
            n = round(k) - 1
        x = n * SNAP_QUANTUM
        return HomeTime(HOME_GUESS_S + x) if x >= SNAP_QUANTUM else 0.0
    return math.floor(t / SNAP_QUANTUM + 1e-6) * SNAP_QUANTUM if t > 0 else 0.0
