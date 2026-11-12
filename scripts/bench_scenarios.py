#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""The browser bench's scenarios (emu/wasm/web/index.html, ?scenario=<id>) in headless Chromium.

    scripts/pyrun -I scripts/bench_scenarios.py [--site build/bench-site] [--shots DIR] [--only id,id]

Serves the bench site that `make bench-site` builds (the page, its module and the default firmware) from a private
local server, opens the page once per scenario with ?scenario=<id> and checks that the page raised no JavaScript error,
that the scenario finished setting up and that the bench is in the state the scenario describes: engine speed, fuel
trim and MAP as both ECUs report them on the link, sparks per ECU, the bench's AFR, and the colour of the ECU and pump
tabs on the Controller's home page (read from the panel's pixels). Also checks an unknown id (a notice, the bench runs
normally), autostart=1 (the bench keeps running), picking a scenario from the list, and the follow-up each description
promises. With --shots, saves a screenshot of the page and of the panel for every scenario. Exit 0 when every check
passes, 1 otherwise.
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
    "rpm-limit": lambda a, b: rpm_is(b, 2950) + firing(a, b, (True, True)),
    "trim-lop": lambda a, b: rpm_is(b, 2400) + [(b["trim"] == [-28, -28], f"trim {b['trim']} (want [-28, -28])"),
                                                 (15.0 <= b["afr"] <= 16.0, f"AFR {b['afr']:.2f} (want 15.0..16.0)")],
}


def follow_up(pg, sid: str, readout) -> list[tuple[bool, str]]:
    """What a scenario's description tells the visitor to try next."""
    if sid == "rpm-limit":            # "the ignition cuts at 3000 RPM"
        pg.evaluate("setRpmPreset(3000); M._s32_wasm_step_quanta(10000)")
        a = readout()
        pg.evaluate("M._s32_wasm_step_quanta(10000)")
        b = readout()
        n = [b["sparks"][k] - a["sparks"][k] for k in (0, 1)]
        return [(max(n) < 10, f"at 3000 RPM: {n} sparks in 1 s (want the ignition cut)")]
    if sid == "map-fail":             # "Select ECU 2 to fuel from the other sensor"
        pg.evaluate("setEcuSelect('2'); M._s32_wasm_step_quanta(30000)")
        b = readout()
        return tabs(b, ecu2="blue") + [(b["afr"] < 13.0, f"ECU 2 selected: AFR {b['afr']:.2f} (want < 13)")]
    if sid == "first-start":          # the first power-up wrote the configuration chips
        n = pg.evaluate("(() => { const p = M._s32_wasm_get_fram_ptr(1); let n = 0;"
                        " for (let i = 0; i < 8192; i++) if (M.HEAPU8[p + i]) n++; return n; })()")
        return [(n > 0, f"ECU 1 configuration chip: {n} non-zero bytes in the first 8 KB (want written)")]
    return []


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--site", default=str(REPO / "build" / "bench-site"), help="the bench site (make bench-site)")
    ap.add_argument("--shots", help="directory for screenshots")
    ap.add_argument("--only", help="comma-separated scenario ids")
    args = ap.parse_args(argv)
    site = pathlib.Path(args.site)
    if not (site / "index.html").is_file() or not (site / "firmware" / "manifest.json").is_file():
        print(f"bench_scenarios: no bench site with default firmware in {site} (make bench-site)")
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
    base = f"http://127.0.0.1:{srv.server_address[1]}/index.html"
    results: list[tuple[bool, str]] = []

    def check(name: str, items: list[tuple[bool, str]]) -> None:
        for ok, what in items:
            results.append((ok, f"{name}: {what}"))
            print(f"  {'ok  ' if ok else 'FAIL'} {name}: {what}", flush=True)

    try:
        with sync_playwright() as p:
            b = browser_job.launch(p)

            def open_page(query: str):
                pg = b.new_page(viewport={"width": 1920, "height": 1400})
                errors: list[str] = []
                pg.on("pageerror", lambda e: errors.append(str(e)))
                pg.goto(base + query)
                pg.wait_for_function("window.S32_SCENARIO && window.S32_SCENARIO.state !== 'running'", timeout=300_000)
                return pg, errors

            # An unknown id: a notice, and the bench runs as without one.
            pg, errors = open_page("?scenario=no-such-scenario")
            ids = pg.evaluate("SCENARIOS.map((s) => s.id)")
            state = pg.evaluate("window.S32_SCENARIO.state")
            note = pg.evaluate("document.getElementById('scn-note').textContent")
            t0 = pg.evaluate("telemetry().time_s")
            pg.wait_for_timeout(1500)
            t1 = pg.evaluate("telemetry().time_s")
            check("unknown id", [(state == "unknown", f"state {state}"), ("no-such-scenario" in note, f"notice '{note}'"),
                                 (t1 > t0 + 0.3, f"bench running ({t0:.2f} -> {t1:.2f} s)"),
                                 (not errors, f"page errors {errors}")])
            check("scenario table", [(sorted(ids) == sorted(EXPECT), f"ids {ids} (checked here: {sorted(EXPECT)})")])
            pg.close()

            only = args.only.split(",") if args.only else ids
            for sid in only:
                pg, errors = open_page(f"?scenario={sid}")
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

            # autostart=1 keeps the bench running; a pick from the list sets the bench up (fuel trim back to 0 %, the
            # link switch back on) and the address bar.
            pg, errors = open_page("?scenario=trim-lop&autostart=1")
            t0 = pg.evaluate("telemetry().time_s")
            pg.wait_for_timeout(1500)
            t1 = pg.evaluate("telemetry().time_s")
            check("autostart=1", [(pg.evaluate("simSpeed") == 1 and t1 > t0 + 0.3, f"running ({t0:.2f} -> {t1:.2f} s)")])
            pg.click("#scn-ecu-silent")
            pg.wait_for_function("window.S32_SCENARIO.id === 'ecu-silent' && window.S32_SCENARIO.state !== 'running'",
                                 timeout=300_000)
            pg.click("#scn-cruise")
            pg.wait_for_function("window.S32_SCENARIO.id === 'cruise' && window.S32_SCENARIO.state !== 'running'",
                                 timeout=300_000)
            r = pg.evaluate(READOUT_JS)
            check("list pick", [(pg.evaluate("window.S32_SCENARIO.state") == "done", "cruise set up"),
                                ("scenario=cruise" in pg.url, f"address {pg.url}"), (r["speed"] == 1, "running"),
                                (r["trim"] == [0, 0], f"trim {r['trim']} (want [0, 0])"),
                                (not errors, f"page errors {errors}")] + tabs(r, ecu1="blue", ecu2="green"))
            pg.close()
            b.close()
    finally:
        srv.shutdown()
    fails = sum(1 for ok, _ in results if not ok)
    print(f"\nbench scenarios: {len(results) - fails} ok, {fails} FAIL")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
