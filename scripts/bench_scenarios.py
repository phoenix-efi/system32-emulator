#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""The scenarios (?scenario=<id>) on the bench page (emu/wasm/web/index.html) and the cockpit page (cockpit.html) in
headless Chromium.

    scripts/pyrun -I scripts/bench_scenarios.py [--site build/bench-site] [--pages index,cockpit] [--shots DIR]
                                                [--only id,id]

Serves the bench site that `make bench-site` builds (the pages, their module and the default firmware) from a private
local server, opens each page once per scenario with ?scenario=<id> and checks that the page raised no JavaScript
error, that the scenario finished setting up and that the bench is in the state the scenario describes: engine speed,
fuel trim and MAP as both ECUs report them on the link, sparks per ECU, the bench's AFR, and the colour of the ECU and
pump tabs on the Controller's home page (read from the panel's pixels).

On the bench page it also checks an unknown id (a notice, the bench runs normally), autostart=1 (the bench keeps
running), picking a scenario from the list, the follow-up each description promises, and that the page's controls
start at BENCH_DEFAULTS (s32_bench.js), the values the cockpit page drives.

On the cockpit page, where an engine model sets RPM and MAP from the throttle and prop levers and the sparks, it also
checks that the switches, the levers and the RPM readout show the bench's state, then goes through every step for the
pilot: shows its text, does what it says with the page's own controls (switches, buttons, knob, levers, the step's
action button) and checks what its "watch" line says: which screen the Controller shows and the tab colors (read from
the panel's pixels), RPM, MAP, trim and AFR from the link, and the sparks per ECU from the bench. For rpm-limit: the
RPM peak at the 2750 RPM limit, a spark cut there, and 2700 RPM held from 3.5 s on. Then an unknown id, ?embed=1 (only
the Controller and the switches, transparent background) and a 390 px wide phone (no sideways scrolling).

With --shots, saves screenshots: on the bench page the page and the panel for every scenario; on the cockpit page
runup, ecu-silent, ecu-select and trim-lop at 1440 and 390 px wide, rpm-limit at 1440, runup in the dark theme
too, the embed, and the Controller alone. Exit 0 when every check passes, 1 otherwise.
"""
from __future__ import annotations

import argparse
import functools
import http.server
import importlib.util
import pathlib
import sys
import threading

REPO = pathlib.Path(__file__).resolve().parent.parent

# Tab colour on the Controller's home page: the label boxes of the four tabs along the top (panel pixels, 320 x 240),
# and the colour most of the label's coloured pixels have. OBSERVED on the stock firmware: blue = the ECU selected to
# fuel, green = running and talking, red = silent, gray = not seen; pumps green (pump 1) or yellow (pump 2) when on,
# gray when off.
READOUT_JS = """
() => {
  const t = telemetry();
  const p = M._s32_wasm_render_rgba();
  const px = new Uint8Array(M.HEAPU8.buffer, p, 320 * 240 * 4);
  const tab = (x0, x1) => {
    const n = {};
    for (let y = 14; y <= 25; y++) for (let x = x0; x <= x1; x++) {
      const i = (y * 320 + x) * 4, r = px[i], g = px[i + 1], b = px[i + 2];
      if ((r | g | b) === 0 || Math.min(r, g, b) > 240) continue;
      const c = r > 200 && g < 80 && b < 80 ? "red" : b > 200 && r < 80 && g < 80 ? "blue"
              : r > 200 && g > 200 && b < 80 ? "yellow" : g > 150 && r < 80 && b < 80 ? "green"
              : Math.abs(r - g) < 24 && Math.abs(g - b) < 24 && r > 90 && r < 210 ? "gray" : "other";
      n[c] = (n[c] || 0) + 1;
    }
    return Object.keys(n).sort((a, b) => n[b] - n[a])[0] || "none";
  };
  return {
    time_s: t.time_s, rpm: t.link.map((l) => l.rpm), map: t.link.map((l) => l.map_inhg), trim: t.trim_pct, afr: t.afr,
    gph: t.fuel_gph, sparks: t.ecu.map((e) => e.coils[0].sparks), powered: t.ecu.map((e) => e.powered),
    tabs: { ecu1: tab(29, 68), ecu2: tab(82, 128), pmp1: tab(148, 190), pmp2: tab(203, 247) },
    speed: simSpeed, note: document.getElementById("scn-note").textContent,
  };
}
"""


def near(v: float, want: float, tol: float) -> bool:
    return abs(v - want) <= tol


def tabs(r: dict, **want: str) -> list[tuple[bool, str]]:
    return [(r["tabs"][k] == v, f"{k} tab {r['tabs'][k]} (want {v})") for k, v in want.items()]


def firing(a: dict, b: dict, which: tuple[bool, bool]) -> list[tuple[bool, str]]:
    out = []
    for k in (0, 1):
        n = b["sparks"][k] - a["sparks"][k]
        out.append(((n > 20) == which[k], f"ECU{k + 1} {n} sparks in 1 s (want {'firing' if which[k] else 'none'})"))
    return out


def rpm_is(r: dict, want: int, tol: int = 20) -> list[tuple[bool, str]]:
    return [(all(near(x, want, tol) for x in r["rpm"]), f"link RPM {r['rpm']} (want {want} +-{tol})")]


# Each scenario's stated state. a = the readout when setup finished, b = after one more second of emulated time.
EXPECT = {
    "first-start": lambda a, b: tabs(b, ecu1="red", ecu2="red") + firing(a, b, (False, False)),
    "runup": lambda a, b: rpm_is(b, 1800) + firing(a, b, (True, True)) +
    tabs(b, ecu1="blue", ecu2="green", pmp1="green", pmp2="gray"),
    "cruise": lambda a, b: rpm_is(b, 2400) + firing(a, b, (True, True)) + tabs(b, ecu1="blue", ecu2="green") +
    [(b["trim"] == [0, 0], f"trim {b['trim']} (want [0, 0])")],
    "takeoff": lambda a, b: rpm_is(b, 2700) + firing(a, b, (True, True)) + tabs(b, ecu1="blue", ecu2="green") +
    [(all(near(x, 29.0, 0.2) for x in b["map"]), f"MAP {b['map']} (want 29.0)")],
    "ecu-silent": lambda a, b: tabs(b, ecu1="red", ecu2="green") + firing(a, b, (True, True)) +
    [(b["powered"] == [True, True], f"powered {b['powered']}")],
    "map-fail": lambda a, b: tabs(b, ecu1="blue", ecu2="green") + firing(a, b, (True, True)) +
    [(b["map"][0] < 10.0 and near(b["map"][1], 24.0, 0.2), f"MAP ECU1/ECU2 {b['map']} (want < 10 / 24.0)")],
    "ecu-select": lambda a, b: rpm_is(b, 2400) + tabs(b, ecu1="green", ecu2="blue") +
    [(b["gph"] > 5.0 and b["afr"] < 13.0, f"fuel {b['gph']:.1f} GPH, AFR {b['afr']:.2f} (want fuelled)")],
    "rpm-limit": lambda a, b: rpm_is(b, 2700) + firing(a, b, (True, True)),
    "trim-lop": lambda a, b: rpm_is(b, 2400) + [(b["trim"] == [-28, -28], f"trim {b['trim']} (want [-28, -28])"),
                                                 (15.0 <= b["afr"] <= 16.0, f"AFR {b['afr']:.2f} (want 15.0..16.0)")],
}


# Which screen the Controller shows, from the panel's pixels. OBSERVED on the stock firmware: a menu has its title in
# yellow at the top left (left of x = 200, where the yellow PMP2 tab of the gauge pages starts) and, when it lists
# items, a blue bar on the selected one. The Main Menu lists five items, the last two on rows 137 to 182; the Tuning Menu
# lists three and leaves those rows black; the Main Menu with the engine running lists none. The gauge pages draw a
# white box under the tab of the ECU whose numbers they show (its top edge on row 5 starts at x = 25 for ECU1, 80 for
# ECU2), whose right edge is at x = 230 on the first page and x = 310 on the second.
SCREEN_JS = """
() => {
  const p = M._s32_wasm_render_rgba();
  const px = new Uint8Array(M.HEAPU8.buffer, p, 320 * 240 * 4);
  const at = (x, y) => { const i = (y * 320 + x) * 4; return [px[i], px[i + 1], px[i + 2]]; };
  const white = ([r, g, b]) => Math.min(r, g, b) > 200;
  const yellow = ([r, g, b]) => r > 200 && g > 200 && b < 80;
  const blue = ([r, g, b]) => b > 200 && r < 80 && g < 80;
  let title = 0;
  for (let y = 0; y < 19; y++) for (let x = 0; x < 200; x++) if (yellow(at(x, y))) title++;
  if (title > 20) {
    let bar = 0, fifth = 0;
    for (let y = 18; y < 60; y++) for (let x = 0; x < 320; x++) if (blue(at(x, y))) bar++;
    for (let y = 137; y <= 182; y++) for (let x = 0; x < 160; x++) if (white(at(x, y))) fifth++;
    const list = bar > 1000;
    return { kind: "menu", title: !list || fifth > 20 ? "main" : "tuning", list };
  }
  let tab = -1;
  for (let x = 0; x < 320 && tab < 0; x++) if (white(at(x, 5))) tab = x;
  const edge = (x) => { let n = 0; for (let y = 40; y < 150; y++) if (white(at(x, y))) n++; return n > 80; };
  return { kind: "gauges", ecu: tab < 0 ? 0 : tab < 50 ? 1 : tab < 100 ? 2 : 0, page: edge(230) ? 1 : edge(310) ? 2 : 0 };
}
"""

# The cockpit page's own view of the bench: switch positions, levers, RPM readout, the step shown.
COCKPIT_JS = """
() => ({
  ign1: document.getElementById("sw-ign1").getAttribute("aria-checked"),
  ign2: document.getElementById("sw-ign2").getAttribute("aria-checked"),
  pump: document.getElementById("sw-pump").dataset.value,
  select: document.getElementById("sw-select").dataset.value,
  throttle: Number(document.getElementById("throttle").getAttribute("aria-valuenow")),
  prop: Number(document.getElementById("prop").getAttribute("aria-valuenow")),
  stop: document.getElementById("set-prop-stop").value,
  rpm: document.getElementById("ro-rpm").textContent,
  step_n: document.getElementById("step-n").textContent,
  say: document.getElementById("step-say").textContent,
  watch: document.getElementById("step-watch").textContent,
  start: !document.getElementById("scn-start").hidden,
})
"""

# The bench page's controls against BENCH_DEFAULTS: the ids whose default value on index.html differs, and the ids a
# scenario resets on index.html that BENCH_DEFAULTS lacks (or the other way round).
DEFAULTS_JS = """
() => {
  const differ = Object.entries(BENCH_DEFAULTS).filter(([id, v]) => {
    const el = document.getElementById(id);
    if (!el) return true;
    const d = el.type === "checkbox" ? el.defaultChecked
            : el.tagName === "SELECT" ? (Array.from(el.options).find((o) => o.defaultSelected) || el.options[0]).value
            : Number(el.defaultValue);
    return d !== v;
  }).map(([id]) => id);
  const keys = Object.keys(BENCH_DEFAULTS);
  const unmatched = SCENARIO_CONTROLS.filter((id) => !keys.includes(id)).concat(keys.filter((id) => !SCENARIO_CONTROLS.includes(id)));
  return { differ, unmatched };
}
"""


def rpm_in(r: dict, lo: int, hi: int) -> list[tuple[bool, str]]:
    return [(all(lo <= x <= hi for x in r["rpm"]), f"link RPM {r['rpm']} (want {lo}..{hi})")]


def limit_push(samples: list) -> list[tuple[bool, str]]:
    """Throttle full, fast, at the 2750 RPM limit: RPM runs up to the limit, the sparks cut there, and the governor then
    holds 2700 (samples every 0.1 s: engine RPM, link RPM, sparks of ECU 1's first coil)."""
    rpm = [x[0] for x in samples]
    cuts = [i for i in range(1, len(samples)) if samples[i][2][0] - samples[i - 1][2][0] <= 1 and rpm[i] > 2500]
    tail = samples[35:]
    return [(max(rpm) >= 2740, f"engine RPM peak {max(rpm)} (want >= 2740)"),
            (bool(cuts), f"spark cut at {[round(0.1 * (i + 1), 1) for i in cuts]} s (want one)"),
            (all(2690 <= x[0] <= 2710 and 2690 <= x[1] <= 2710 for x in tail),
             f"from 3.5 s: engine RPM {[x[0] for x in tail]}, link {[x[1] for x in tail]} (want 2690..2710)")]


def screen_is(scr: dict, **want) -> list[tuple[bool, str]]:
    return [(all(scr.get(k) == v for k, v in want.items()), f"screen {scr} (want {want})")]


def afr_in(r: dict, lo: float, hi: float, what: str = "AFR") -> list[tuple[bool, str]]:
    return [(lo <= r["afr"] <= hi, f"{what} {r['afr']:.2f} (want {lo}..{hi})")]


def trim_is(r: dict, want: int) -> list[tuple[bool, str]]:
    return [(r["trim"] == [want, want], f"trim {r['trim']} (want [{want}, {want}])")]


def map_near(r: dict, k: int, want: float, tol: float) -> list[tuple[bool, str]]:
    return [(near(r["map"][k], want, tol), f"ECU{k + 1} MAP {r['map'][k]} (want {want} +-{tol})")]


# Each pilot step on the cockpit page (SCENARIOS[..].pilot in s32_bench.js), in order: what the test does with the
# page's controls, and what the step's "watch" line says, as checks. Actions: ("flip", switch id), ("press", button,
# times) with 1 = MENU, 2 = TUNE, 3 = MONITOR, ("knob", clicks) through the arrow keys, ("act",) the step's action
# button, ("lever", throttle | prop, % or a page expression) the End key for 100 %, else the page's lever setter;
# None for a step that only asks the pilot to look. An optional third item: {"settle": s, "samples": n}. s.b = the
# readout after the step, s.a = 1.5 s earlier, s.r0 = before the step, s.samples = engine RPM, link RPM and sparks
# every 0.1 s between them, s.scr = the Controller's screen, s.c = the page's own view.
GAUGES = {"kind": "gauges"}
CRUISE_THROTTLE = "throttleFor(24.0, 2400) * 100"
PILOT = {
    "first-start": [
        (None, lambda s: tabs(s.b, ecu1="red", ecu2="red") + rpm_is(s.b, 0, 0) + firing(s.a, s.b, (False, False))),
        (("press", 1, 1), lambda s: screen_is(s.scr, kind="menu", title="main", list=True)),
        (("press", 3, 1), lambda s: screen_is(s.scr, **GAUGES)),
        (("act",), lambda s: tabs(s.b, ecu1="blue", ecu2="green")),
    ],
    "runup": [
        (("flip", "sw-ign1"), lambda s: rpm_in(s.b, 1680, 1730) + tabs(s.b, ecu1="red") + firing(s.a, s.b, (False, True))),
        (("flip", "sw-ign1"), lambda s: rpm_is(s.b, 1800) + tabs(s.b, ecu1="blue")),
        (("flip", "sw-ign2"), lambda s: rpm_in(s.b, 1680, 1730) + tabs(s.b, ecu2="red") + firing(s.a, s.b, (True, False))),
        (("flip", "sw-ign2"), lambda s: rpm_is(s.b, 1800) + tabs(s.b, ecu2="green")),
        (("flip", "sw-pump"), lambda s: tabs(s.b, pmp1="gray", pmp2="yellow")),
        (("flip", "sw-select"), lambda s: tabs(s.b, ecu1="green", ecu2="blue")),
    ],
    "cruise": [
        (("flip", "sw-ign1"), lambda s: rpm_is(s.b, 2400) + tabs(s.b, ecu1="red") + firing(s.a, s.b, (False, True))),
        (("flip", "sw-ign1"), lambda s: tabs(s.b, ecu1="blue")),
        (("knob", -5), lambda s: trim_is(s.b, -5) + afr_in(s.r0, 10.9, 11.3, "AFR before") + afr_in(s.b, 11.5, 11.9)),
        (("knob", 5), lambda s: trim_is(s.b, 0) + afr_in(s.b, 10.9, 11.3)),
        (("press", 3, 1), lambda s: screen_is(s.scr, kind="gauges", ecu=1, page=2)),
        (("press", 3, 1), lambda s: screen_is(s.scr, kind="gauges", ecu=2, page=1)),
    ],
    "takeoff": [
        (None, lambda s: rpm_is(s.b, 2700) + map_near(s.b, 0, 29.0, 0.3)),
        (("press", 1, 1), lambda s: screen_is(s.scr, kind="menu", title="main", list=False)),
        (("press", 3, 1), lambda s: screen_is(s.scr, **GAUGES)),
        (("press", 2, 1), lambda s: screen_is(s.scr, kind="menu", title="tuning", list=True) + rpm_is(s.b, 2700)),
        (("press", 3, 1), lambda s: screen_is(s.scr, **GAUGES)),
        (("lever", "throttle", CRUISE_THROTTLE), lambda s: map_near(s.b, 0, 24.0, 0.3) + rpm_in(s.b, 2500, 2600)),
        (("lever", "prop", "propFor(2400) * 100"), lambda s: rpm_is(s.b, 2400)),
    ],
    "ecu-silent": [
        (None, lambda s: tabs(s.b, ecu1="red") + firing(s.a, s.b, (True, True))),
        (("flip", "sw-select"), lambda s: tabs(s.b, ecu1="red", ecu2="blue")),
        (("press", 3, 2), lambda s: screen_is(s.scr, kind="gauges", ecu=2, page=1)),
    ],
    "map-fail": [
        (None, lambda s: [(s.b["map"][0] < 10.0, f"ECU1 MAP {s.b['map'][0]} (want < 10)")] + afr_in(s.b, 19.9, 20.1)),
        (("flip", "sw-select"), lambda s: tabs(s.b, ecu2="blue") + afr_in(s.b, 10.5, 11.6)),
        (("press", 3, 2), lambda s: screen_is(s.scr, kind="gauges", ecu=2, page=1) + map_near(s.b, 1, 24.0, 0.3)),
    ],
    "ecu-select": [
        (None, lambda s: tabs(s.b, ecu1="green", ecu2="blue") + firing(s.a, s.b, (True, True))),
        (("flip", "sw-ign2"), lambda s: tabs(s.b, ecu2="red") + firing(s.a, s.b, (True, False)) + rpm_is(s.b, 2400)),
        (("flip", "sw-ign2"), lambda s: tabs(s.b, ecu2="blue")),
        (("flip", "sw-select"), lambda s: tabs(s.b, ecu1="blue", ecu2="green")),
    ],
    "rpm-limit": [
        (("lever", "throttle", 100), lambda s: limit_push(s.samples), {"settle": 0, "samples": 45}),
    ],
    "trim-lop": [
        (None, lambda s: trim_is(s.b, -28) + afr_in(s.b, 15.0, 16.0)),
        (("knob", 28), lambda s: trim_is(s.b, 0) + afr_in(s.b, 10.9, 11.4)),
    ],
}

# The cockpit page's switches and levers (throttle %, prop %) once a scenario is set up; cruise: the prop at 2400 RPM,
# the throttle at 24 inHg.
SWITCHES = {"ign1": "true", "ign2": "true", "pump": "1", "select": "1"}
LEVERS = {"first-start": (0, 100), "runup": (25, 100), "takeoff": (100, 100), "rpm-limit": (25, 100)}
CRUISE_LEVERS = (54.9, 57.1)


def switches_show(sid: str, c: dict, r: dict) -> list[tuple[bool, str]]:
    want = dict(SWITCHES, select="2" if sid == "ecu-select" else "1")
    got = {k: c[k] for k in want}
    thr, prop = LEVERS.get(sid, CRUISE_LEVERS)
    out = [(got == want, f"switches {got} (want {want})"),
           (near(c["throttle"], thr, 0.6) and near(c["prop"], prop, 0.6),
            f"throttle {c['throttle']} %, prop {c['prop']} % (want {thr}, {prop})")]
    out.append((near(int(c["rpm"]), r["rpm"][0], 20), f"RPM readout {c['rpm']} (link {r['rpm']})"))
    if sid == "rpm-limit":
        out.append((c["stop"] == "fine", f"low-pitch stop {c['stop']} (want fine)"))
    return out


# The engine model's RPM, the link RPM and the sparks per ECU, every 0.1 s for n samples.
def samples(pg, n: int) -> list:
    return [pg.evaluate("runBench(0.1), [Math.round(eng.rpm), telemetry().link[0].rpm, "
                        "telemetry().ecu.map((e) => e.coils[0].sparks)]") for _ in range(n)]


# A scenario's state on the cockpit page where its engine model makes it differ from the bench page.
EXPECT_COCKPIT = {
    "rpm-limit": lambda pg, a, b: rpm_in(b, 2020, 2060) + firing(a, b, (True, True)),   # run-up on a fine stop
}


class Step:
    """One pilot step's result: r0 before it, a and b one second apart after it, scr the screen, c the page."""


def do_step(pg, action, readout, opts: dict | None = None) -> Step:
    """Do one pilot step's action with the cockpit page's controls, let the bench run, and read the result."""
    opts = opts or {}
    s = Step()
    s.r0 = readout()
    settle = 2.0
    if action is None:
        pass
    elif action[0] == "flip":
        pg.click("#" + action[1])
    elif action[0] == "press":
        for _ in range(action[2]):
            pg.click(f"#cc-btn-{action[1]}")
            pg.evaluate("runBench(1.5)")
    elif action[0] == "knob":
        pg.focus("#cc-knob")
        for _ in range(abs(action[1])):
            pg.keyboard.press("ArrowRight" if action[1] > 0 else "ArrowLeft")
        settle = abs(action[1]) * 0.135 + 2.0
    elif action[0] == "lever":
        if action[2] == 100:
            pg.focus("#" + action[1])
            pg.keyboard.press("End")
        else:
            pg.evaluate(f"interact(), setLever({action[1]!r}, {action[2]})")
    elif action[0] == "act":
        pg.click("#step-act")
        pg.wait_for_function("!scenarios.busy()", timeout=120_000)
    settle = opts.get("settle", settle)
    if settle:
        pg.evaluate(f"runBench({settle})")
    s.a = readout()
    s.samples = samples(pg, opts.get("samples", 15))
    s.b = readout()
    s.scr = pg.evaluate(SCREEN_JS)
    s.c = pg.evaluate(COCKPIT_JS)
    return s


def follow_up(pg, sid: str, readout) -> list[tuple[bool, str]]:
    """What a scenario's description tells the visitor to try next."""
    if sid == "rpm-limit":            # "Move the RPM slider to 2750: the ignition cuts."
        pg.evaluate("setRpmPreset(2750); M._s32_wasm_step_quanta(10000)")
        a = readout()
        pg.evaluate("M._s32_wasm_step_quanta(10000)")
        b = readout()
        n = [b["sparks"][k] - a["sparks"][k] for k in (0, 1)]
        return [(max(n) < 10, f"at 2750 RPM: {n} sparks in 1 s (want the ignition cut)")]
    if sid == "map-fail":             # "Select ECU 2 to fuel from the other sensor"
        pg.evaluate("setEcuSelect('2'); M._s32_wasm_step_quanta(30000)")
        b = readout()
        return tabs(b, ecu2="blue") + [(b["afr"] < 13.0, f"ECU 2 selected: AFR {b['afr']:.2f} (want < 13)")]
    if sid == "first-start":          # the first power-up wrote the configuration chips
        n = pg.evaluate("(() => { const p = M._s32_wasm_get_fram_ptr(1); let n = 0;"
                        " for (let i = 0; i < 8192; i++) if (M.HEAPU8[p + i]) n++; return n; })()")
        return [(n > 0, f"ECU 1 configuration chip: {n} non-zero bytes in the first 8 KB (want written)")]
    return []


def index_page(open_page, check, only, shots) -> None:
    """The bench page: an unknown id, every scenario, autostart=1, a pick from the list, the defaults."""
    # An unknown id: a notice, and the bench runs as without one.
    pg, errors = open_page("index.html?scenario=no-such-scenario")
    ids = pg.evaluate("SCENARIOS.map((s) => s.id)")
    state = pg.evaluate("window.S32_SCENARIO.state")
    note = pg.evaluate("document.getElementById('scn-note').textContent")
    t0 = pg.evaluate("telemetry().time_s")
    pg.wait_for_timeout(1500)
    t1 = pg.evaluate("telemetry().time_s")
    check("unknown id", [(state == "unknown", f"state {state}"), ("no-such-scenario" in note, f"notice '{note}'"),
                         (t1 > t0 + 0.3, f"bench running ({t0:.2f} -> {t1:.2f} s)"),
                         (not errors, f"page errors {errors}")])
    check("scenario table", [(sorted(ids) == sorted(EXPECT), f"ids {ids} (checked here: {sorted(EXPECT)})"),
                             (sorted(ids) == sorted(PILOT), f"ids {ids} (pilot steps checked here: {sorted(PILOT)})")])
    d = pg.evaluate(DEFAULTS_JS)
    check("bench defaults", [(not d["differ"], f"controls whose default differs from BENCH_DEFAULTS: {d['differ']}"),
                             (not d["unmatched"], f"controls in only one of SCENARIO_CONTROLS, BENCH_DEFAULTS: "
                                                  f"{d['unmatched']}")])
    pg.close()

    for sid in only or ids:
        pg, errors = open_page(f"index.html?scenario={sid}")
        state = pg.evaluate("window.S32_SCENARIO")
        readout = functools.partial(pg.evaluate, READOUT_JS)
        a = readout()
        pg.evaluate("M._s32_wasm_step_quanta(10000); renderLcdFrame(M._s32_wasm_render_rgba()); "
                    "renderTelemetry(telemetry())")
        r = readout()
        print(f"{sid}: {r}", flush=True)
        if shots:
            pg.screenshot(path=str(shots / f"{sid}.png"), full_page=True)
            pg.locator("#lcd").screenshot(path=str(shots / f"{sid}-panel.png"))
        check(sid, [(state.get("state") == "done", f"setup {state}"), (a["speed"] == 0, "paused after setup"),
                    (not errors, f"page errors {errors}")] + EXPECT[sid](a, r) + follow_up(pg, sid, readout))
        pg.close()

    # autostart=1 keeps the bench running; a pick from the list sets the bench up (fuel trim back to 0 %, the link
    # switch back on, rpm-limit's RPM limit setting back to its value before) and the address bar.
    pg, errors = open_page("index.html?scenario=trim-lop&autostart=1")
    t0 = pg.evaluate("telemetry().time_s")
    pg.wait_for_timeout(1500)
    t1 = pg.evaluate("telemetry().time_s")
    check("autostart=1", [(pg.evaluate("simSpeed") == 1 and t1 > t0 + 0.3, f"running ({t0:.2f} -> {t1:.2f} s)")])
    for sid in ("rpm-limit", "ecu-silent"):
        pg.click(f"#scn-{sid}")
        pg.wait_for_function(f"window.S32_SCENARIO.id === '{sid}' && window.S32_SCENARIO.state !== 'running'",
                             timeout=300_000)
    pg.click("#scn-cruise")
    pg.wait_for_function("window.S32_SCENARIO.id === 'cruise' && window.S32_SCENARIO.state !== 'running'",
                         timeout=300_000)
    r = pg.evaluate(READOUT_JS)
    slot7 = pg.evaluate("[1, 2].map((k) => { const p = M._s32_wasm_get_fram_ptr(k) + 0x1480 + 6 * 16; let s = '';"
                        " for (let i = 0; i < 16 && M.HEAPU8[p + i]; i++) s += String.fromCharCode(M.HEAPU8[p + i]);"
                        " return s; })")
    check("list pick", [(pg.evaluate("window.S32_SCENARIO.state") == "done", "cruise set up"),
                        ("scenario=cruise" in pg.url, f"address {pg.url}"), (r["speed"] == 1, "running"),
                        (r["trim"] == [0, 0], f"trim {r['trim']} (want [0, 0])"),
                        (slot7 == ["3000", "3000"], f"ECU setting 7 {slot7} (want back at 3000)"),
                        (not errors, f"page errors {errors}")] + tabs(r, ecu1="blue", ecu2="green"))
    pg.close()


def cockpit_page(open_page, check, only, shots) -> None:
    """The cockpit page: every scenario and its pilot steps, an unknown id, the embed, a phone, the screenshots."""
    for sid in only or list(PILOT):
        pg, errors = open_page(f"cockpit.html?scenario={sid}")
        name = f"cockpit {sid}"
        state = pg.evaluate("window.S32_SCENARIO")
        readout = functools.partial(pg.evaluate, READOUT_JS)
        a = readout()
        pg.evaluate("runBench(1)")
        r = readout()
        c = pg.evaluate(COCKPIT_JS)
        print(f"{name}: {r}", flush=True)
        expect = EXPECT_COCKPIT[sid](pg, a, r) if sid in EXPECT_COCKPIT else EXPECT[sid](a, r)
        check(name, [(state.get("state") == "done", f"setup {state}"), (a["speed"] == 0, "paused after setup"),
                     (c["start"], "Start shown while paused")] + expect + switches_show(sid, c, r))
        pilot = pg.evaluate(f"SCENARIOS.find((s) => s.id === {sid!r}).pilot")
        check(name, [(len(pilot) == len(PILOT[sid]), f"{len(pilot)} pilot steps, {len(PILOT[sid])} checked here")])
        pg.click("#scn-start")
        for k, (item, step) in enumerate(zip(PILOT[sid], pilot)):
            action, want = item[0], item[1]
            c = pg.evaluate(COCKPIT_JS)
            shown = c["say"] == step["say"] and c["watch"] == step["watch"]
            s = do_step(pg, action, readout, item[2] if len(item) > 2 else None)
            print(f"{name} step {k + 1}: {s.b} {s.scr}", flush=True)
            check(f"{name} step {k + 1}", [(shown, f"shows '{c['say']}'")] + want(s))
            pg.click("#step-next")
        c = pg.evaluate(COCKPIT_JS)
        check(name, [(c["say"] == "Done", f"after the last step: '{c['say']}'"),
                     (not errors, f"page errors {errors}")])
        pg.close()

    # An unknown id: a notice, and the emulator runs as without one.
    pg, errors = open_page("cockpit.html?scenario=no-such-scenario")
    state = pg.evaluate("window.S32_SCENARIO.state")
    note = pg.evaluate("document.getElementById('scn-note').textContent")
    t0 = pg.evaluate("telemetry().time_s")
    pg.wait_for_timeout(1500)
    t1 = pg.evaluate("telemetry().time_s")
    check("cockpit unknown id", [(state == "unknown", f"state {state}"),
                                 ("no-such-scenario" in note, f"notice '{note}'"),
                                 (t1 > t0 + 0.3, f"running ({t0:.2f} -> {t1:.2f} s)"),
                                 (not errors, f"page errors {errors}")])
    pg.close()

    # ?embed=1: the Controller and the switches only, on a transparent background, at a guide page's iframe width
    # and at a phone's.
    for width in (760, 390):
        pg, errors = open_page("cockpit.html?scenario=runup&autostart=1&embed=1", width=width, height=480)
        pg.wait_for_timeout(1200)
        vis = {sel: pg.locator(sel).is_visible() for sel in (".top", ".scn", "#bench", "#cc", "#switches")}
        want = {".top": False, ".scn": False, "#bench": False, "#cc": True, "#switches": True}
        bg = pg.evaluate("getComputedStyle(document.body).backgroundColor")
        sw = pg.evaluate("[document.documentElement.scrollWidth, innerWidth]")
        r = pg.evaluate(READOUT_JS)
        check(f"cockpit embed {width}", [(pg.evaluate("window.S32_SCENARIO.state") == "done", "runup set up"),
                                         (vis == want, f"visible {vis}"),
                                         (bg in ("rgba(0, 0, 0, 0)", "transparent"), f"background {bg}"),
                                         (sw[0] <= sw[1], f"page width {sw[0]} in a {sw[1]} px window"),
                                         (not errors, f"page errors {errors}")] + tabs(r, ecu1="blue", ecu2="green"))
        if shots and width == 760:
            pg.screenshot(path=str(shots / "cockpit-embed.png"), full_page=True)
        pg.close()

    # Desktop and phone, light and dark: no sideways scrolling, and the screenshots.
    views = [(sid, "desktop", 1440, "light") for sid in ("runup", "ecu-silent", "ecu-select", "trim-lop", "rpm-limit")]
    views += [(sid, "phone", 390, "light") for sid in ("runup", "ecu-silent", "ecu-select", "trim-lop")]
    views += [("runup", "desktop", 1440, "dark"), ("runup", "phone", 390, "dark")]
    for sid, kind, width, scheme in views if shots else [("runup", "phone", 390, "dark")]:
        pg, errors = open_page(f"cockpit.html?scenario={sid}&autostart=1", width=width,
                               height=844 if kind == "phone" else 1000, color_scheme=scheme)
        pg.wait_for_timeout(1500)
        sw = pg.evaluate("[document.documentElement.scrollWidth, innerWidth]")
        cc = pg.locator("#cc").bounding_box()
        check(f"cockpit {sid} {kind} {scheme}", [(sw[0] <= sw[1], f"page width {sw[0]} in a {sw[1]} px window"),
                                                  (kind == "desktop" or cc["width"] >= width - 40,
                                                   f"Controller {cc['width']:.0f} px wide"),
                                                  (not errors, f"page errors {errors}")])
        if shots:
            suffix = "" if scheme == "light" else "-dark"
            pg.screenshot(path=str(shots / f"cockpit-{sid}-{kind}{suffix}.png"), full_page=True)
            if (sid, kind, scheme) == ("runup", "desktop", "light"):
                pg.locator("#cc").screenshot(path=str(shots / "cockpit-instrument.png"))
        pg.close()


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--site", default=str(REPO / "build" / "bench-site"), help="the bench site (make bench-site)")
    ap.add_argument("--pages", default="index,cockpit", help="which pages: index, cockpit or both (comma-separated)")
    ap.add_argument("--shots", help="directory for screenshots")
    ap.add_argument("--only", help="comma-separated scenario ids")
    args = ap.parse_args(argv)
    site = pathlib.Path(args.site)
    pages = args.pages.split(",")
    for page in pages:
        if page not in ("index", "cockpit") or not (site / f"{page}.html").is_file():
            print(f"bench_scenarios: no {page}.html in {site} (make bench-site)")
            return 1
    if not (site / "firmware" / "manifest.json").is_file():
        print(f"bench_scenarios: no default firmware in {site} (make bench-site)")
        return 1
    shots = pathlib.Path(args.shots) if args.shots else None
    if shots:
        shots.mkdir(parents=True, exist_ok=True)
    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        print("bench_scenarios: playwright not installed (run through scripts/pyrun: uv.lock)")
        return 3
    spec = importlib.util.spec_from_file_location("browser_job", REPO / "scripts" / "browser_job.py")
    browser_job = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(browser_job)

    class Quiet(http.server.SimpleHTTPRequestHandler):
        def log_message(self, *a):
            pass
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), functools.partial(Quiet, directory=str(site)))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}/"
    results: list[tuple[bool, str]] = []

    def check(name: str, items: list[tuple[bool, str]]) -> None:
        for ok, what in items:
            results.append((ok, f"{name}: {what}"))
            print(f"  {'ok  ' if ok else 'FAIL'} {name}: {what}", flush=True)

    only = args.only.split(",") if args.only else None
    try:
        with sync_playwright() as p:
            b = browser_job.launch(p)

            def open_page(path: str, width: int = 1920, height: int = 1400, color_scheme: str = "light"):
                pg = b.new_page(viewport={"width": width, "height": height}, color_scheme=color_scheme)
                errors: list[str] = []
                pg.on("pageerror", lambda e: errors.append(str(e)))
                pg.goto(base + path)
                pg.wait_for_function("window.S32_SCENARIO && window.S32_SCENARIO.state !== 'running'", timeout=300_000)
                return pg, errors

            if "index" in pages:
                index_page(open_page, check, only, shots)
            if "cockpit" in pages:
                cockpit_page(open_page, check, only, shots)
            b.close()
    finally:
        srv.shutdown()
    fails = sum(1 for ok, _ in results if not ok)
    print(f"\nbench scenarios: {len(results) - fails} ok, {fails} FAIL")
    return 1 if fails else 0

if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
