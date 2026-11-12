#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Emulator conformance gate: the same stimulus must give the same canonical trace (emu/common/s32trace.h) on every
build, every platform and every run.

    python3 scripts/conformance.py [--firmware-dir DIR] [--tier fast|full] [--only ID ...] [--builds B,B,...]
                                   [--remote USER@HOST] [--remote-builds B,B,...] [--jobs N] [--keep DIR]
    python3 scripts/conformance.py --synthetic        # no firmware needed: generated test images (make check)

Stimuli: the emulator's own cases (tests/conformance/cases.json) and the benchmark scenarios (tests/bench/scenarios.json),
in the case format of scripts/s32case.py (home-page times "{H+x}" in knobs placed by the emulator, run lengths fixed at
5 + x s; --home N fixes every home time at N). --provider FILE.py takes the cases from another module instead (a
firmware project's behavioural cases; docs/conformance.md, "Case providers"). Each case's inputs (FRAM images, SD
cards, knobs) are prepared once by the provider's prepare() and every build runs on a fresh copy of the same inputs. A
case that takes another case's output (a FRAM image, a flash readback) gets the reference run's output.

Builds (names in --builds; the first is the reference every other run is compared with):
    native          emu/pair/build/s32pair_shim, one thread per MCU (the reference build)
    native-again    the same binary again (determinism)
    native-serial   the same binary with S32_SERIAL=1: the three MCUs one after another on one thread
    native-scribble the same binary with the C library filling fresh heap memory with a pattern (uninitialised reads)
    wasm-node       emu/pair/s32pair.c compiled to WebAssembly (emu/wasm build/s32pair_node.js), in Node
    wasm-bench      the browser bench's module (emu/wasm build/s32_wasm.js) through its exported API, in Node
                    (emu/wasm/test/run_job.mjs); cases it cannot express (ECU SD cards, per-ECU flash images, replay)
                    are SKIPped with the reason
    browser         the same module in headless Chromium (scripts/browser_job.py, Playwright from uv.lock through
                    scripts/pyrun); compared by the trace's SHA-256 only (the page keeps no trace text)
    snapshot        boot once (emu/common/s32snap.h), as a case runner does: cases that start the same way are grouped
                    (S32_STIMULUS), one case per group runs from power-on writing a snapshot where each other case's
                    stimulus begins, the others start from those snapshots (S32_SNAPSHOT_IN); every run is compared,
                    the leaders too (writing snapshots must change nothing)
    snapshot-self   each case writes snapshots at 1/4, 1/2 and 3/4 of its length and is restored from each with its own
                    knobs: 4 runs per case
    snapshot-wasm   snapshot-self with the WebAssembly build (wasm-node): snapshots written and restored in WebAssembly
--remote runs --remote-builds on a second machine over ssh (the tree is copied there and built natively; the
WebAssembly files are copied, not rebuilt) and compares them with the local reference: the cross-platform check.

Verdict: every compared run must give a byte-identical trace and the same exit code; the first differing trace line
is printed for each mismatch. Exit 0 = conformant (SKIPs allowed), 1 = a mismatch or a failed run, 2 = usage error.
Without firmware the firmware cases SKIP (exit 0); --synthetic runs generated images on every local build instead.
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import gzip
import hashlib
import json
import math
import os
import pathlib
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import time

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))
import s32case as K  # noqa: E402

# The case provider: scripts/s32case.py unless --provider names another module (load_provider). It supplies
# load_cases(tier, only, home), prepare(group, case, fw, dir, done) -> (env, secs, ...) and needs_boot_code(group, case).
L = K

# CONFORMANCE_PAIR: another native binary as the "native" builds (e.g. one built with -ftrivial-auto-var-init=pattern)
PAIR = pathlib.Path(os.environ.get("CONFORMANCE_PAIR") or REPO / "emu" / "pair" / "build" / "s32pair_shim")
NODE_PAIR = REPO / "emu" / "wasm" / "build" / "s32pair_node.js"
BENCH_MOD = REPO / "emu" / "wasm" / "build" / "s32_wasm.js"
RUN_JOB = REPO / "emu" / "wasm" / "test" / "run_job.mjs"
SCENARIOS = K.SCENARIOS
# Log options of s32pair that the trace replaces (the conformance runs do not need them).
LOG_KEYS = ("S32_SPARKLOG", "S32_INJLOG", "S32_STATUSLOG", "S32_LINKLOG", "S32_LINKRAW", "S32_CT_CONSOLE",
            "S32_SNAPDIR", "S32_SNAP_MS")
# Environment options the browser bench's module has no equivalent for (a case using one SKIPs on wasm-bench).
BENCH_UNSUPPORTED = ("S32_ECU1_SD", "S32_ECU2_SD", "S32_CT_PFLASH", "S32_ECU1_PFLASH", "S32_ECU2_PFLASH", "S32_REPLAY",
                     "S32_XPLANE", "S32_NOIDLESKIP", "S32_TRIM_LAG_MS", "ECU_CONSOLE", "S32_IOCOV", "S32_READHIST")
ALL_BUILDS = ("native", "native-again", "native-serial", "native-scribble", "wasm-node", "wasm-bench", "browser",
              "snapshot", "snapshot-self", "snapshot-wasm")
SNAP_BUILDS = ("snapshot", "snapshot-self", "snapshot-wasm")
SELF_POINTS = (0.25, 0.5, 0.75)
HOME_SELF_POINTS = ("{H}", "{H+1}")    # snapshot-self, cases with home-page times: also at the home page and 1 s after
HOME_SECS_BASE = 5.0                   # run length of a case with home-page times: HOME_SECS_BASE + x for "{H+x}"


def host_tag() -> str:
    return {"Darwin": "mac", "Linux": "linux"}.get(platform.system(), platform.system().lower())


def node_bin() -> str | None:
    """Node, not the emsdk's bundled copy (as emu/wasm/Makefile picks it)."""
    if os.environ.get("NODE"):
        return os.environ["NODE"]
    for p in os.environ.get("PATH", "").split(os.pathsep):
        c = pathlib.Path(p) / "node"
        if c.is_file() and os.access(c, os.X_OK) and "emsdk" not in str(c):
            return str(c)
    return None


def build_env(build: str) -> dict[str, str]:
    if build == "native-serial":
        return {"S32_SERIAL": "1"}
    if build == "native-scribble":       # fresh heap memory filled with a pattern instead of whatever was there
        return {"MallocPreScribble": "1", "MallocScribble": "1"} if platform.system() == "Darwin" else {"MALLOC_PERTURB_": "165"}
    return {}


def safe_name(cid: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", cid)


# ---- cases ------------------------------------------------------------------------------------------------------

def load_provider(path: pathlib.Path):
    """A case provider module from a file (--provider): it must define load_cases and prepare; needs_boot_code is
    optional (no case needs the boot region)."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("conformance_provider", path)
    if spec is None or spec.loader is None:
        raise SystemExit(f"conformance: cannot load provider {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    for name in ("load_cases", "prepare"):
        if not hasattr(mod, name):
            raise SystemExit(f"conformance: provider {path} has no {name}()")
    return mod


def home_case(c: dict, home: float | None) -> dict:
    """A case for the gate. home None: its "{H+x}" knob times stay for the emulator to place at the home page it
    detects; the run length (which the browser bench module takes as a number) is fixed at HOME_SECS_BASE + x. home a
    number: every home-page time replaced by home + x (the older fixed-home mode)."""
    if home is not None:
        return K.home_sub(c, home)
    cc = json.loads(json.dumps(c))
    st = cc.get("stimulus", {})
    if isinstance(st.get("secs"), str):
        st["secs"] = K.home_sub(st["secs"], HOME_SECS_BASE)
    return cc


def load_cases(tier: str, only: list[str] | None, home: float | None, bench_only: bool = False) -> list[dict]:
    """Every stimulus from the provider, dependencies first: {"id", "group" (dict), "case" (dict, home times per
    home_case), "deps" (case ids)}. bench_only: the benchmark scenarios alone (scripts/bench.py, --no-laws)."""
    src = K if bench_only else L
    cases = []
    for c in src.load_cases(tier, only, None):
        if bench_only and not c["id"].startswith("bench/"):
            continue
        cc = home_case(c["case"], home)
        cc.setdefault("name", c["case"].get("name"))
        cases.append(dict(c, case=cc))
    ids = {c["id"] for c in cases}
    missing = sorted({d for c in cases for d in c["deps"] if d not in ids})
    if missing:                                         # a dependency outside the selection is pulled in
        extra = [c for c in src.load_cases(tier, None, None) if c["id"] in missing]
        cases = [dict(c, case=home_case(c["case"], home)) for c in extra] + cases
    ordered, done = [], set()
    while len(ordered) < len(cases):
        progress = False
        for c in cases:
            if c["id"] not in done and all(d in done for d in c["deps"]):
                ordered.append(c)
                done.add(c["id"])
                progress = True
        if not progress:
            raise SystemExit("conformance: circular or unknown case dependencies")
    return ordered


def prepare_case(c: dict, fw: dict, work: pathlib.Path, ref_dirs: dict[str, pathlib.Path]) -> dict:
    """Inputs for one case in work/cases/<name>/ and its manifest entry (paths relative to the run dir as {D})."""
    d = work / "cases" / safe_name(c["id"])
    if d.exists():
        shutil.rmtree(d)
    d.mkdir(parents=True)
    done = {dep.split("/", 1)[1]: ref_dirs[dep] for dep in c["deps"] if dep in ref_dirs}
    nbc = getattr(L, "needs_boot_code", None)
    why = nbc(c["group"], c["case"]) if fw.get("app_start") and nbc else None
    gone = [dep for dep in c["deps"] if dep not in ref_dirs]
    if why or gone:                                    # no boot region, or a case it takes its input from did not run
        return {"id": c["id"], "safe": safe_name(c["id"]),
                "skip": why or f"needs the output of {', '.join(gone)}, which did not run"}
    try:
        env, secs = L.prepare(c["group"], c["case"], fw, d, done)[:2]
    except Exception as e:                              # noqa: BLE001 -- a provider's "cannot run here" (K.Missing)
        if type(e).__name__ != "Missing":
            raise
        return {"id": c["id"], "safe": safe_name(c["id"]), "skip": str(e)}
    for k in LOG_KEYS:
        env.pop(k, None)
    env["S32_PPM"] = str(d / "screen.ppm")
    for p in d.glob("cardsrc_*"):
        shutil.rmtree(p)
    env = {k: v.replace(str(d), "{D}") for k, v in env.items()}
    return {"id": c["id"], "safe": safe_name(c["id"]), "env": env, "secs": float(secs),
            "anchored": any(K.HOME_RE.search(v) for v in env.values())}


# ---- runs -------------------------------------------------------------------------------------------------------

def file_sha(p: pathlib.Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def run_dir(entry: dict, work: pathlib.Path, tag: str, label: str) -> tuple[pathlib.Path, dict[str, str]]:
    """A fresh copy of a case's prepared inputs in work/runs/<tag>-<label>/<case>/ and its environment there."""
    rd = work / "runs" / f"{tag}-{label}" / entry["safe"]
    if rd.exists():
        shutil.rmtree(rd)
    shutil.copytree(work / "cases" / entry["safe"], rd)
    return rd, {k: v.replace("{D}", str(rd)) for k, v in entry["env"].items()}


def run_one(entry: dict, build: str, work: pathlib.Path, fwfiles: dict[str, str], tag: str, keep_all: bool,
            extra: dict[str, str] | None = None, label: str | None = None) -> dict:
    """Run one case on one build in work/runs/<tag>-<label or build>/<case>/. Returns {"rc", "sha", "wall", "dir"} or
    {"skip": why}. extra: more environment (snapshot options; "{R}" is the run directory)."""
    if "skip" in entry:
        return {"skip": entry["skip"]}
    rd, env = run_dir(entry, work, tag, label or build)
    env.update(build_env(build))
    env.update({k: v.replace("{R}", str(rd)) for k, v in (extra or {}).items()})
    trace = rd / "trace.txt"
    env["S32_TRACE"] = str(trace)
    secs = entry["secs"]
    args = [fwfiles["ct"], fwfiles["bfl"], fwfiles["pfl"], f"{secs:g}"]
    node = node_bin()
    if build.startswith("native"):
        cmd = [str(PAIR)] + args
    elif build == "wasm-node":
        if not node or not NODE_PAIR.exists():
            return {"skip": "no Node or no emu/wasm/build/s32pair_node.js (make -C emu/wasm node)"}
        cmd = [node, str(NODE_PAIR)] + args
    elif build in ("wasm-bench", "browser"):
        if not BENCH_MOD.exists() or (build == "wasm-bench" and not node):
            return {"skip": "no Node or no emu/wasm/build/s32_wasm.js (make -C emu/wasm)"}
        bad = [k for k in BENCH_UNSUPPORTED if env.get(k)]
        if bad:
            return {"skip": f"the browser bench module has no {', '.join(bad)}"}
        knobs = harness_knobs()
        job = {"ct": fwfiles["ct"], "bfl": None if fwfiles["bfl"] == "none" else fwfiles["bfl"], "pfl": fwfiles["pfl"],
               "fram": {k: env.get(e) for k, e in (("ct", "S32_CT_FRAM"), ("e1", "S32_ECU1_FRAM"), ("e2", "S32_ECU2_FRAM"))},
               "ct_sd": env.get("S32_CT_SD"), "knobs": [[k, v] for k, v in env.items() if k in knobs and v],
               "secs": secs, "trace": str(trace)}
        (rd / "job.json").write_text(json.dumps(job, indent=1))
        cmd = ([node, str(RUN_JOB), str(rd / "job.json")] if build == "wasm-bench" else
               [str(REPO / "scripts" / "pyrun"), "-I", str(REPO / "scripts" / "browser_job.py"), str(rd / "job.json")])
    else:
        raise SystemExit(f"unknown build {build}")
    t0 = time.monotonic()
    try:
        r = subprocess.run(cmd, env=dict(os.environ, **env), cwd=rd, capture_output=True, text=True,
                           timeout=max(600.0, 20.0 * secs))
        rc, out = r.returncode, r.stdout + r.stderr
    except subprocess.TimeoutExpired as e:
        rc, out = -9, f"timeout after {e.timeout:.0f} s\n"
    wall = time.monotonic() - t0
    (rd / "out.txt").write_text(out)
    res = {"rc": rc, "wall": wall, "secs": secs, "dir": str(rd)}
    if build == "browser":                              # the page keeps no trace text: the module's own SHA-256 of it
        try:
            j = json.loads(out.strip().splitlines()[0])
            if j.get("rc") == 0 and j.get("trace_sha256"):
                res.update(rc=0, sha=j["trace_sha256"], browser=j.get("browser"))
        except (ValueError, IndexError):
            pass
        return res
    if trace.exists():
        res["sha"] = file_sha(trace)
        with open(trace, "rb") as f, gzip.open(str(trace) + ".gz", "wb", compresslevel=3) as g:
            shutil.copyfileobj(f, g)
        trace.unlink()
    if not keep_all:                                    # the big inputs; the trace and the FRAM / flash outputs stay
        for p in rd.glob("card_*.img"):
            p.unlink()
    return res


_KNOBS: set[str] | None = None


def harness_knobs() -> set[str]:
    """The harness knob names (s32pair --knobs, minus s32pair's own front-end options)."""
    global _KNOBS
    if _KNOBS is None:
        out = subprocess.run([str(PAIR), "--knobs"], capture_output=True, text=True).stdout
        _KNOBS = {ln.split()[0] for ln in out.splitlines() if ln.strip() and "(s32pair option)" not in ln}
    return _KNOBS


def run_builds(manifest: list[dict], builds: list[str], work: pathlib.Path, fwfiles: dict[str, str], tag: str,
               jobs: int, keep_all: bool, results: dict, log=print) -> None:
    items = [(e, b) for b in builds for e in manifest if (e["id"], f"{tag}:{b}") not in results]
    items.sort(key=lambda eb: -eb[0].get("secs", 0))
    with cf.ThreadPoolExecutor(max_workers=max(1, jobs)) as ex:
        futs = {ex.submit(run_one, e, b, work, fwfiles, tag, keep_all): (e, b) for e, b in items}
        for f in cf.as_completed(futs):
            e, b = futs[f]
            r = f.result()
            results[(e["id"], f"{tag}:{b}")] = r
            if "skip" not in r:
                log(f"    {tag}:{b:15s} {e['id']:48s} rc {r['rc']} {r['wall']:6.1f} s  {r.get('sha', 'no trace')[:16]}")


# ---- snapshots (boot once) ---------------------------------------------------------------------------------------

def run_snapshots(manifest: list[dict], builds: list[str], work: pathlib.Path, fwfiles: dict[str, str], tag: str,
                  jobs: int, keep_all: bool, results: dict, log=print) -> None:
    """The snapshot builds (SNAP_BUILDS): restored runs whose traces must equal the reference's."""
    runnable = [e for e in manifest if "skip" not in e]
    stims: dict[str, dict] = {}
    if runnable:                                         # what each case's knobs set up (the grouping input)
        def stim(e: dict):
            rd, env = run_dir(e, work, tag, "stimulus")
            return e["id"], K.stimulus(env, e["secs"], {"ct": fwfiles["ct"], "bfl": fwfiles["bfl"], "pfl": fwfiles["pfl"]}, rd)
        with cf.ThreadPoolExecutor(max_workers=max(1, jobs)) as ex:
            for cid, st in ex.map(stim, runnable):
                if st:
                    stims[cid] = st
    snapdir = work / "snapshots" / tag
    snapdir.mkdir(parents=True, exist_ok=True)

    def batch(jobs_list: list[tuple[dict, str, str, dict, str]]) -> None:
        """Run (entry, build, label, extra env, result key) items; results[(id, key)]."""
        with cf.ThreadPoolExecutor(max_workers=max(1, jobs)) as ex:
            futs = {ex.submit(run_one, e, b, work, fwfiles, tag, keep_all, x, lab): (e, key) for e, b, lab, x, key in jobs_list}
            for f in cf.as_completed(futs):
                e, key = futs[f]
                r = f.result()
                results[(e["id"], f"{tag}:{key}")] = r
                if "skip" not in r:
                    log(f"    {tag}:{key:22s} {e['id']:48s} rc {r['rc']} {r['wall']:6.1f} s  {r.get('sha', 'no trace')[:16]}")

    def not_restorable(e: dict, key: str) -> bool:
        st = stims.get(e["id"])
        if st and st.get("snapshot"):
            return False
        results[(e["id"], f"{tag}:{key}")] = {"skip": "no snapshot: " + (st.get("why") if st else "stimulus not readable")}
        return True

    for build in [b for b in builds if b in ("snapshot-self", "snapshot-wasm")]:
        exe = "native" if build == "snapshot-self" else "wasm-node"
        if exe == "wasm-node" and (not node_bin() or not NODE_PAIR.exists()):
            for e in manifest:
                results[(e["id"], f"{tag}:{build}")] = {"skip": "no Node or no emu/wasm/build/s32pair_node.js"}
            continue
        first, second = [], []
        for e in manifest:
            if "skip" in e or not_restorable(e, build):
                continue
            pts = [f"{t:.4f}" for t in
                   sorted({math.floor(e["secs"] * f / K.SNAP_QUANTUM + 1e-6) * K.SNAP_QUANTUM for f in SELF_POINTS} - {0.0})]
            if e.get("anchored"):
                pts += list(HOME_SELF_POINTS)
            files = [snapdir / f"{build}-{e['safe']}-{safe_name(t)}.snap" for t in pts]
            first.append((e, exe, build, {"S32_SNAPSHOT_OUT": ",".join(f"{p}@{t}" for p, t in zip(files, pts))}, build))
            for p, t in zip(files, pts):
                second.append((e, exe, f"{build}-{safe_name(t)}", {"S32_SNAPSHOT_IN": str(p)}, f"{build}@{t}"))
        batch(first)
        batch(second)
        if not keep_all:
            for e, _, _, x, _ in second:
                pathlib.Path(x["S32_SNAPSHOT_IN"]).unlink(missing_ok=True)

    if "snapshot" in builds:
        groups: dict[str, list[dict]] = {}
        for e in manifest:
            if "skip" in e or not_restorable(e, "snapshot"):
                continue
            groups.setdefault(stims[e["id"]]["static"], []).append(e)
        leaders, members, nsnap = [], [], 0
        agree: dict[tuple[str, str], float] = {}
        for es in groups.values():
            left = list(es)
            while left:
                best = None
                for lead in left:
                    gains = {}
                    for m in left:
                        if m is lead:
                            continue
                        k = tuple(sorted((lead["id"], m["id"])))
                        if k not in agree:
                            agree[k] = K.agree_until(stims[lead["id"]]["items"], stims[m["id"]]["items"])
                        t = K.snap_point(agree[k], stims[lead["id"]]["guard"], lead["secs"], m["secs"])
                        if t >= K.SNAP_MIN_GAIN:
                            gains[m["id"]] = (m, t)
                    if best is None or sum(t for _, t in gains.values()) > best[1]:
                        best = (lead, sum(t for _, t in gains.values()), gains)
                lead, _, gains = best
                by_t: dict[float, pathlib.Path] = {}
                for m, t in sorted(gains.values(), key=lambda mt: mt[1]):
                    if t not in by_t:
                        if len(by_t) >= 64:
                            continue
                        by_t[t] = snapdir / f"group-{nsnap:04d}-{safe_name(K.snap_spec(t))}.snap"
                        nsnap += 1
                    members.append((m, "native", "snapshot", {"S32_SNAPSHOT_IN": str(by_t[t])}, "snapshot"))
                out = ",".join(f"{p}@{K.snap_spec(t)}" for t, p in sorted(by_t.items()))
                leaders.append((lead, "native", "snapshot", {"S32_SNAPSHOT_OUT": out} if out else {}, "snapshot"))
                left = [x for x in left if x is not lead and not (x["id"] in gains and gains[x["id"]][1] in by_t)]
        batch(leaders)
        batch(members)
        log(f"    {tag}:snapshot: {sum(1 for x in leaders if x[3])} shared boots, {len(members)} restored runs")
        if not keep_all:
            for _, _, _, x, _ in members:
                pathlib.Path(x["S32_SNAPSHOT_IN"]).unlink(missing_ok=True)


# ---- comparison -------------------------------------------------------------------------------------------------

def open_trace(d: pathlib.Path):
    p = d / "trace.txt.gz"
    return gzip.open(p, "rt", errors="replace") if p.exists() else open(d / "trace.txt", errors="replace")


def first_diff(a: pathlib.Path, b: pathlib.Path) -> str:
    try:
        with open_trace(a) as fa, open_trace(b) as fb:
            prev: list[str] = []
            for n, (la, lb) in enumerate(zip(fa, fb), 1):
                if la != lb:
                    ctx = "".join(f"           {x}" for x in prev[-2:])
                    return f"first difference at trace line {n}:\n{ctx}      ref: {la.rstrip()}\n      run: {lb.rstrip()}"
                prev.append(la)
            return "one trace is a prefix of the other (a run ended early)"
    except OSError as e:
        return f"trace unreadable: {e}"


# ---- remote (cross-platform) ------------------------------------------------------------------------------------

def ssh(host: str, cmd: str, check: bool = True, capture: bool = True) -> subprocess.CompletedProcess:
    r = subprocess.run(["ssh", "-o", "BatchMode=yes", host, cmd], capture_output=capture, text=True)
    if check and r.returncode:
        raise SystemExit(f"remote command failed ({r.returncode}): {cmd}\n{(r.stderr or '')[-2000:]}")
    return r


def rsync(src: list[str], dst: str, extra: list[str] | None = None) -> None:
    r = subprocess.run(["rsync", "-a"] + (extra or []) + src + [dst], capture_output=True, text=True)
    if r.returncode:
        raise SystemExit(f"rsync failed: {r.stderr[-2000:]}")


def remote_run(host: str, rdir: str, work: pathlib.Path, fw: dict, manifest: list[dict], builds: list[str], jobs: int,
               results: dict, renv: str) -> tuple[str, str]:
    """Copy the tree, the firmware (private, mode 700) and the prepared cases to host:rdir, build natively there,
    run `builds` with --worker and collect the results. Returns the remote tag and the remote compiler line."""
    if not rdir.startswith("/"):
        rdir = ssh(host, "echo $HOME").stdout.strip() + "/" + rdir
    ssh(host, f"mkdir -p {rdir}/src {rdir}/fw {rdir}/work && chmod 700 {rdir} {rdir}/fw")
    rsync(["--exclude=.git", "--exclude=build/", "--exclude=__pycache__",
           f"{REPO}/"], f"{host}:{rdir}/src/")
    rsync([f"{REPO}/emu/wasm/build/"], f"{host}:{rdir}/src/emu/wasm/build/",
          ["--include=*.js", "--include=*.wasm", "--exclude=*"])
    rsync([str(fw[k]) for k in ("ct", "bfl", "pfl") if str(fw[k]) != "none"], f"{host}:{rdir}/fw/")   # none: app start
    ssh(host, f"chmod 600 {rdir}/fw/*")
    rsync([f"{work}/cases/", ], f"{host}:{rdir}/work/cases/")
    (work / "manifest.json").write_text(json.dumps(manifest))
    rsync([str(work / "manifest.json")], f"{host}:{rdir}/work/")
    ssh(host, f"cd {rdir}/src && {renv} nice -n 19 make -s emu >/dev/null 2>{rdir}/build.log || (tail -30 {rdir}/build.log; exit 1)")
    names = {k: pathlib.Path(fw[k]).name for k in ("ct", "bfl", "pfl")}
    cmd = (f"cd {rdir}/src && {renv} nice -n 19 python3 scripts/conformance.py --worker {rdir}/work --worker-fw "
           f"{rdir}/fw/{names['ct']},{'none' if names['bfl'] == 'none' else rdir + '/fw/' + names['bfl']},{rdir}/fw/{names['pfl']} "
           f"--builds {','.join(builds)} --jobs {jobs}")
    r = ssh(host, cmd, check=False, capture=False)
    if r.returncode:
        raise SystemExit(f"remote worker failed ({r.returncode})")
    rsync([f"{host}:{rdir}/work/results.json"], str(work / "remote_results.json"))
    rem = json.loads((work / "remote_results.json").read_text())
    for row in rem["results"]:
        results[(row["id"], row["build"])] = row["result"] | {"remote": True}
    return rem["tag"], rdir


def fetch_remote_trace(host: str, rdir: str, work: pathlib.Path, res: dict) -> pathlib.Path:
    rel = pathlib.Path(res["dir"]).relative_to(f"{rdir}/work") if res["dir"].startswith(rdir) else None
    local = work / "fetched" / (rel or pathlib.Path(res["dir"]).name)
    local.mkdir(parents=True, exist_ok=True)
    rem = f"{rdir}/work/{rel}" if rel else res["dir"]
    rsync([f"{host}:{rem}/trace.txt.gz", f"{host}:{rem}/out.txt"], f"{local}/")
    return local


# ---- synthetic images (no firmware) -----------------------------------------------------------------------------

def synthetic_firmware(d: pathlib.Path) -> dict[str, str]:
    """Tiny generated images that exercise every front end without any vendor firmware. Controller (PIC24): set
    PORTA to outputs and toggle it between 0x5555 and 0xAAAA with REPEAT/NOP delays. ECUs (PIC32 boot flash at the
    reset vector): set TRISA = 0 and count on LATA with a busy-wait delay. Configuration words select the emulator's
    clocks (Controller 8 MHz crystal / 2 -> 96 MHz PLL, ECU 8 MHz / 2 x 20 = 80 MHz, watchdogs off)."""
    d.mkdir(parents=True, exist_ok=True)
    ct = bytearray(b"\xff" * ((0x2AC00 // 2) * 3))

    def ctw(pc: int, word: int) -> None:
        i = (pc // 2) * 3
        ct[i:i + 3] = word.to_bytes(3, "little")
    prog = {0x000: 0x040200, 0x002: 0x000000,                      # GOTO 0x200
            0x200: 0x200000, 0x202: 0x881600,                      # MOV #0, W0; MOV W0, TRISA
            0x204: 0x255550, 0x206: 0x881620,                      # MOV #0x5555, W0; MOV W0, LATA
            0x208: 0x0903E8, 0x20A: 0x000000,                      # REPEAT #1000; NOP
            0x20C: 0x2AAAA0, 0x20E: 0x881620,                      # MOV #0xAAAA, W0; MOV W0, LATA
            0x210: 0x0903E8, 0x212: 0x000000,                      # REPEAT #1000; NOP
            0x214: 0x040204, 0x216: 0x000000,                      # GOTO 0x204
            0x2ABFC: 0x9BFD, 0x2ABFE: 0xFF7F}                      # CONFIG2, CONFIG1
    for pc, w in prog.items():
        ctw(pc, w)
    bfl = bytearray(b"\xff" * 0x3000)
    code = [0x3C08BF88, 0x35086000, 0xAD000000, 0x24090000,          # t0 = &TRISA; TRISA = 0; t1 = 0
            0x25290001, 0xAD090020, 0x240A03E8,                      # loop: t1++; LATA = t1; t2 = 1000
            0x254AFFFF, 0x1540FFFE, 0x00000000,                      # dly: t2--; bne t2, dly; nop
            0x1000FFF9, 0x00000000]                                  # b loop; nop
    for i, w in enumerate(code):
        bfl[4 * i:4 * i + 4] = w.to_bytes(4, "little")
    bfl[0x2FF4:0x2FF8] = (0xFFF8FFD9).to_bytes(4, "little")          # DEVCFG2: /2 x20 /1
    bfl[0x2FF8:0x2FFC] = (0xFF7FFEFB).to_bytes(4, "little")          # DEVCFG1: PRIPLL, HS, watchdog off
    out = {"ct": d / "synth_prog.bin", "bfl": d / "synth_bflash.bin", "pfl": d / "synth_pflash.bin"}
    out["ct"].write_bytes(bytes(ct))
    out["bfl"].write_bytes(bytes(bfl))
    out["pfl"].write_bytes(b"\xff" * 0x80000)
    return {k: str(v) for k, v in out.items()}


SYNTH_CASES = [
    {"id": "synthetic/idle", "env": {}, "secs": 1.0},
    {"id": "synthetic/engine", "env": {"S32_RPM": "2400", "S32_START": "0.2", "S32_CYL": "6", "S32_AN": "5=900"}, "secs": 1.0},
    {"id": "synthetic/faults", "env": {"S32_RPM": "1800", "S32_START": "0.1", "S32_CRANK_GLITCH": "1:0.5",
                                       "S32_ECU_POWER": "2:off@0.4,2:on@0.7", "S32_TPS_NOISE": "40", "S32_XTAL_PPM": "1=50,2=-30"},
     "secs": 1.0},
]


# ---- main -------------------------------------------------------------------------------------------------------

def tool_versions() -> dict[str, str]:
    def first(cmd: list[str]) -> str:
        try:
            r = subprocess.run(cmd, capture_output=True, text=True)
            return (r.stdout or r.stderr).splitlines()[0].strip()
        except (OSError, IndexError):
            return "absent"
    built = REPO / "emu" / "wasm" / "build" / "emcc-version.txt"     # written by `make wasm-emcc` (Makefile)
    return {"cc": first([os.environ.get("CC", "cc"), "--version"]), "emcc": first(["emcc", "--version"]),
            "emcc_built": built.read_text().strip() if built.is_file() else "unknown (not built through make)",
            "node": first([node_bin() or "node", "--version"])}


def worker(a: argparse.Namespace) -> int:
    work = pathlib.Path(a.worker)
    manifest = json.loads((work / "manifest.json").read_text())
    ct, bfl, pfl = a.worker_fw.split(",")
    fwfiles = {"ct": ct, "bfl": bfl, "pfl": pfl}
    results: dict = {}
    tag = host_tag()
    wb = [b for b in a.builds.split(",") if b]
    run_builds(manifest, [b for b in wb if b not in SNAP_BUILDS], work, fwfiles, tag, a.jobs, False, results)
    if any(b in SNAP_BUILDS for b in wb):
        run_snapshots(manifest, wb, work, fwfiles, tag, a.jobs, False, results)
    rows = [{"id": cid, "build": b, "result": r} for (cid, b), r in results.items()]
    (work / "results.json").write_text(json.dumps({"tag": tag, "tools": tool_versions(), "results": rows}))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--firmware-dir", type=pathlib.Path, default=None)
    ap.add_argument("--synthetic", action="store_true", help="generated test images instead of firmware")
    ap.add_argument("--tier", choices=("fast", "full"), default="fast")
    ap.add_argument("--only", nargs="*", default=None, help="case ids (prefix match), e.g. idle bench/")
    ap.add_argument("--no-laws", "--bench-only", dest="no_laws", action="store_true", help="only the benchmark scenarios")
    ap.add_argument("--provider", type=pathlib.Path, default=None,
                    help="a Python file with load_cases() and prepare(): take the cases from it (docs/conformance.md)")
    ap.add_argument("--home", type=float, default=None,
                    help="fixed home-page time for {H} in the cases (default: the emulator places {H} at the home page "
                         "it detects; run lengths use 5 s)")
    ap.add_argument("--builds", default="native,native-again,native-serial,wasm-node,wasm-bench")
    ap.add_argument("--remote", default=None, help="user@host for the cross-platform runs")
    ap.add_argument("--remote-dir", default="s32-conformance")
    ap.add_argument("--remote-builds", default="native,native-again,native-serial,native-scribble,wasm-node,wasm-bench")
    ap.add_argument("--remote-jobs", type=int, default=6)
    ap.add_argument("--remote-env", default="", help="VAR=value ... set for the remote build and runs (e.g. PATH with "
                    "Node, CPATH / LIBRARY_PATH for zlib)")
    ap.add_argument("--jobs", type=int, default=1, help="local emulators at once (default 1)")
    ap.add_argument("--keep", type=pathlib.Path, default=None, help="keep every run directory here")
    ap.add_argument("--json", type=pathlib.Path, default=None, help="write the results here")
    ap.add_argument("--worker", default=None, help=argparse.SUPPRESS)
    ap.add_argument("--worker-fw", default=None, help=argparse.SUPPRESS)
    a = ap.parse_args()
    global L
    if a.provider:
        L = load_provider(a.provider)
    if a.worker:
        return worker(a)
    builds = [b for b in a.builds.split(",") if b]
    for b in builds + [b for b in a.remote_builds.split(",") if b]:
        if b not in ALL_BUILDS:
            print(f"conformance: unknown build {b} (known: {', '.join(ALL_BUILDS)})")
            return 2
    if not PAIR.exists():
        print(f"FAIL: {PAIR} not built (make emu)")
        return 1
    if a.firmware_dir is None and os.environ.get("S32_FIRMWARE_DIR") and not a.synthetic:
        a.firmware_dir = pathlib.Path(os.environ["S32_FIRMWARE_DIR"]).expanduser()
    work = a.keep.resolve() if a.keep else pathlib.Path(tempfile.mkdtemp(prefix="conformance-"))
    work.mkdir(parents=True, exist_ok=True)
    tag = host_tag()
    t_start = time.monotonic()
    tools = tool_versions()
    print(f"conformance: builds {', '.join(builds)} on {tag} ({tools['cc']}; emcc {tools['emcc']}; node {tools['node']})")
    print(f"conformance: WebAssembly files built by: {tools['emcc_built']}")

    if a.synthetic:
        fwfiles = synthetic_firmware(work / "synthetic")
        manifest = []
        for c in SYNTH_CASES:
            d = work / "cases" / safe_name(c["id"])
            d.mkdir(parents=True, exist_ok=True)
            manifest.append({"id": c["id"], "safe": safe_name(c["id"]), "env": dict(c["env"]), "secs": c["secs"]})
        fw = None
    else:
        if a.firmware_dir is None:
            print("  SKIP: no firmware (set S32_FIRMWARE_DIR or pass --firmware-dir; none ships with this repository); "
                  "--synthetic runs generated images instead")
            return 0
        fw, why = K.find_firmware(a.firmware_dir.resolve())
        if fw is None:
            print(f"FAIL: firmware directory unusable: {why}")
            return 1
        fwfiles = {k: str(fw[k]) for k in ("ct", "bfl", "pfl")}
        cases = load_cases(a.tier, a.only, a.home, bench_only=a.no_laws)
        print(f"  {len(cases)} cases ({a.provider or 'tests/conformance/cases.json'}, {a.tier} tier, benchmark scenarios); home page "
              + (f"at {a.home:g} s" if a.home is not None else "placed by the emulator"))
        manifest = []

    results: dict = {}
    ref = f"{tag}:{builds[0]}"
    if not a.synthetic:
        # reference runs in dependency order: a case may take a previous case's output as its input
        ref_dirs: dict[str, pathlib.Path] = {}
        for c in cases:
            e = prepare_case(c, fw, work, ref_dirs)
            manifest.append(e)
            r = run_one(e, builds[0], work, fwfiles, tag, bool(a.keep))
            results[(e["id"], ref)] = r
            if "dir" in r:
                ref_dirs[c["id"]] = pathlib.Path(r["dir"])
            print(f"    {ref:20s} {e['id']:48s} " + (f"SKIP {r['skip']}" if "skip" in r else
                                                        f"rc {r['rc']} {r['wall']:6.1f} s  {r.get('sha', 'no trace')[:16]}"))
    (work / "manifest.json").write_text(json.dumps(manifest))
    rtag = None
    remote_thread, remote_err = None, []
    if a.remote and not a.synthetic:                    # the second machine runs while this one does its own builds
        print(f"  remote {a.remote}: builds {a.remote_builds}")
        rres: dict = {}

        def remote_job() -> None:
            nonlocal rtag
            try:
                rtag, a.remote_dir = remote_run(a.remote, a.remote_dir, work, fw, manifest,
                                                [b for b in a.remote_builds.split(",") if b], a.remote_jobs, rres,
                                                a.remote_env)
            except BaseException as e:                  # noqa: BLE001 (reported below; SystemExit included)
                remote_err.append(str(e))
        import threading
        remote_thread = threading.Thread(target=remote_job)
        remote_thread.start()
    run_builds(manifest, [b for b in builds if b not in SNAP_BUILDS], work, fwfiles, tag, a.jobs, bool(a.keep), results)
    if any(b in SNAP_BUILDS for b in builds):
        run_snapshots(manifest, builds, work, fwfiles, tag, a.jobs, bool(a.keep), results)
    if remote_thread:
        remote_thread.join()
        if remote_err:
            print(f"conformance: FAIL: remote runs: {remote_err[0]}")
            return 1
        results.update(rres)

    # ---- verdict
    mism, failed, ok, skipped = [], [], 0, 0
    labels = {}
    for (cid, b) in results:
        host, name = b.split(":", 1)
        labels[b] = ("snapshot" if name.startswith("snapshot") else "cross-build" if name.startswith("wasm") else
                     "determinism" if host == tag else "cross-platform")
    for e in manifest:
        r0 = results.get((e["id"], ref), {})
        for b in sorted({b for (cid, b) in results if cid == e["id"] and b != ref}):
            r = results[(e["id"], b)]
            if "skip" in r or "skip" in r0:
                skipped += 1
                continue
            if r0.get("sha") is None or r.get("sha") is None:
                failed.append((e["id"], b, f"no trace (rc {r0.get('rc')} / {r.get('rc')}; see {r.get('dir')}/out.txt)"))
                continue
            if r["sha"] == r0["sha"] and r["rc"] == r0["rc"]:
                ok += 1
                continue
            rd = pathlib.Path(r["dir"])
            if r.get("remote"):
                rd = fetch_remote_trace(a.remote, a.remote_dir, work, r)
            why = first_diff(pathlib.Path(r0["dir"]), rd) if r["sha"] != r0["sha"] else f"exit code {r0['rc']} vs {r['rc']}"
            mism.append((e["id"], b, labels.get(b, ""), why))
    skips_ref = sum(1 for e in manifest if "skip" in results.get((e["id"], ref), {"skip": 1}))
    print(f"\nconformance: reference {ref}; {ok} identical, {len(mism)} DIFFERENT, {len(failed)} failed, "
          f"{skipped} skipped comparisons; {skips_ref} case(s) not runnable on this firmware")
    for (cid, b, kind, why) in mism:
        print(f"  DIFFERENT {cid} on {b} ({kind}): {why}")
    for (cid, b, why) in failed:
        print(f"  FAILED    {cid} on {b}: {why}")
    skip_reasons: dict[str, int] = {}
    for (cid, b), r in results.items():
        if "skip" in r:
            skip_reasons[f"{b}: {r['skip']}"] = skip_reasons.get(f"{b}: {r['skip']}", 0) + 1
    for k, n in sorted(skip_reasons.items()):
        print(f"  skipped x{n}: {k}")
    if a.json:
        a.json.write_text(json.dumps({"reference": ref, "tools": tools, "remote": rtag,
                                      "results": [{"id": c, "build": b, **{k: v for k, v in r.items() if k != 'dir'}}
                                                  for (c, b), r in results.items()]}, indent=1))
    total = time.monotonic() - t_start
    if mism or failed:
        print(f"conformance: FAIL ({total:.0f} s); run directories kept in {work}")
        return 1
    if not a.keep:
        shutil.rmtree(work, ignore_errors=True)
    print(f"conformance: PASS ({total:.0f} s; WebAssembly built by {tools['emcc_built']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
