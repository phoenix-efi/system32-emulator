#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Run one emulator job (the emu/wasm/test/run_job.mjs job format) in headless Chromium on the browser bench's module.

    scripts/pyrun -I scripts/browser_job.py job.json

Serves emu/wasm/build/s32_wasm.{js,wasm}, emu/wasm/test/job_page.html and the job's input files from a private
local server (127.0.0.1, a free port), opens the page in headless Chromium (Playwright) and prints the page's result
as one JSON line (rc, secs, wall_s, x_real_time, trace_sha256, user_agent). Used by scripts/bench.py (speed) and
scripts/conformance.py (build "browser": the trace's SHA-256 against the native run).
"""
from __future__ import annotations

import functools
import http.server
import json
import os
import pathlib
import shutil
import sys
import tempfile
import threading

REPO = pathlib.Path(__file__).resolve().parent.parent


def launch(p):
    """Headless Chromium: Playwright's own, else the newest headless shell already in Playwright's browser cache (a
    Playwright release newer than the cached browsers), else the full Chromium there. PLAYWRIGHT_BROWSERS_PATH (the
    flake's Chromium on Linux, which has the full browser and no headless shell) is searched first."""
    try:
        return p.chromium.launch(headless=True)
    except Exception as e:                                       # noqa: BLE001 (Playwright raises its own Error)
        if "Executable doesn't exist" not in str(e):
            raise
    caches = [pathlib.Path(os.environ["PLAYWRIGHT_BROWSERS_PATH"])] if os.environ.get("PLAYWRIGHT_BROWSERS_PATH") else []
    for cache in caches + [pathlib.Path.home() / "Library" / "Caches" / "ms-playwright", pathlib.Path.home() / ".cache" / "ms-playwright"]:
        for pat in ("chromium_headless_shell-*/chrome-headless-shell-*/chrome-headless-shell",
                    "chromium-*/chrome-*/Chromium.app/Contents/MacOS/Chromium", "chromium-*/chrome-*/chrome"):
            found = sorted(cache.glob(pat), key=lambda x: int(x.parts[len(cache.parts)].split("-")[-1]), reverse=True)
            if found:
                return p.chromium.launch(headless=True, executable_path=str(found[0]))
    raise SystemExit("no Chromium for Playwright (playwright install chromium)")


def main(argv: list[str]) -> int:
    job = json.loads(pathlib.Path(argv[0]).read_text())
    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        print(json.dumps({"rc": 3, "error": "playwright not installed (run through scripts/pyrun: uv.lock)"}))
        return 3
    with tempfile.TemporaryDirectory(prefix="browser-job-") as td:
        d = pathlib.Path(td)
        for f in ("s32_wasm.js", "s32_wasm.wasm"):
            shutil.copy(REPO / "emu" / "wasm" / "build" / f, d / f)
        shutil.copy(REPO / "emu" / "wasm" / "test" / "job_page.html", d / "index.html")
        page_job = dict(job)
        for key in ("ct", "bfl", "pfl", "ct_sd"):
            if job.get(key):
                shutil.copy(job[key], d / f"in_{key}")
                page_job[key] = f"in_{key}"
        page_job["fram"] = {}
        for k, v in (job.get("fram") or {}).items():
            if v:
                shutil.copy(v, d / f"in_fram_{k}")
                page_job["fram"][k] = f"in_fram_{k}"
        (d / "job.json").write_text(json.dumps(page_job))

        class Quiet(http.server.SimpleHTTPRequestHandler):
            def log_message(self, *a):
                pass
        srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), functools.partial(Quiet, directory=str(d)))
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        try:
            with sync_playwright() as p:
                b = launch(p)
                page = b.new_page()
                page.goto(f"http://127.0.0.1:{srv.server_address[1]}/index.html")
                page.wait_for_function("window.JOB_RESULT !== undefined", timeout=max(600_000, int(job["secs"] * 30_000)))
                r = page.evaluate("window.JOB_RESULT")
                r["browser"] = b.version
                b.close()
        finally:
            srv.shutdown()
    print(json.dumps(r))
    return int(r.get("rc", 2))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
