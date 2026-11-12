#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Emulator speed benchmark: simulated seconds per wall-clock second for the fixed scenarios in
tests/bench/scenarios.json (boot to the home page, 2400 RPM cruise for 30 s, a crank-fault
run), on each build:

    native         emu/pair/build/s32pair_shim, one thread per MCU (three cores)
    native-serial  the same binary, S32_SERIAL=1: the three MCUs on one thread (one core)
    wasm-node      emu/pair/s32pair.c compiled to WebAssembly, in Node (one core)
    wasm-bench     the browser bench's module (emu/wasm/build/s32_wasm.js) in Node (one core)
    browser        the browser bench's module in headless Chromium (Playwright; scripts/browser_job.py)

    python3 scripts/bench.py [--firmware-dir DIR] [--builds B,...] [--repeat 3] [--only cruise ...]
                             [--remote USER@HOST --remote-env '...'] [--json OUT]

The figure is the emulator's own (its loop only: no process start, firmware load or WebAssembly compile), the median
of --repeat runs, one emulator at a time on this machine. Prints a Markdown table with the CPU, compiler, flags and
the load average. No firmware: SKIP (exit 0).
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import platform
import re
import shutil
import statistics
import subprocess
import sys
import tempfile

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))
import conformance as C  # noqa: E402

BUILDS = ("native", "native-serial", "wasm-node", "wasm-bench", "browser")


def machine() -> dict[str, str]:
    m = {"os": f"{platform.system()} {platform.release()} {platform.machine()}"}
    try:
        if platform.system() == "Darwin":
            m["cpu"] = subprocess.run(["sysctl", "-n", "machdep.cpu.brand_string"], capture_output=True, text=True).stdout.strip()
            m["cores"] = subprocess.run(["sysctl", "-n", "hw.ncpu"], capture_output=True, text=True).stdout.strip()
        else:
            info = pathlib.Path("/proc/cpuinfo").read_text()
            m["cpu"] = re.search(r"model name\s*:\s*(.*)", info).group(1).strip()
            m["cores"] = str(os.cpu_count())
    except (OSError, AttributeError):
        pass
    m.update(C.tool_versions())
    mk = (REPO / "emu" / "pair" / "Makefile").read_text()
    m["native_flags"] = " ".join(x.strip() for x in re.findall(r"^CFLAGS\s*\+?\??=\s*(.*)$", mk, re.M))
    m["wasm_flags"] = "-std=gnu11 -O3 -flto -ffp-contract=off"
    m["load"] = " ".join(f"{x:.1f}" for x in os.getloadavg())
    return m


def run_speed(entry: dict, build: str, work: pathlib.Path, fwfiles: dict[str, str]) -> dict:
    """One timed run without the trace. Returns {"x": simulated s per wall s, ...} or {"skip": why}."""
    if "skip" in entry:
        return {"skip": entry["skip"]}
    rd = work / "runs" / build / entry["safe"]
    if rd.exists():
        shutil.rmtree(rd)
    shutil.copytree(work / "cases" / entry["safe"], rd)
    env = {k: v.replace("{D}", str(rd)) for k, v in entry["env"].items()}
    env.update(C.build_env(build))
    secs = entry["secs"]
    args = [fwfiles["ct"], fwfiles["bfl"], fwfiles["pfl"], f"{secs:g}"]
    node = C.node_bin()
    if build in ("wasm-bench", "browser"):
        bad = [k for k in C.BENCH_UNSUPPORTED if env.get(k)]
        if bad:
            return {"skip": f"the browser bench module has no {', '.join(bad)}"}
        knobs = C.harness_knobs()
        job = {"ct": fwfiles["ct"], "bfl": fwfiles["bfl"], "pfl": fwfiles["pfl"],
               "fram": {k: env.get(e) for k, e in (("ct", "S32_CT_FRAM"), ("e1", "S32_ECU1_FRAM"), ("e2", "S32_ECU2_FRAM"))},
               "ct_sd": env.get("S32_CT_SD"), "knobs": [[k, v] for k, v in env.items() if k in knobs and v],
               "secs": secs, "trace": None}
        (rd / "job.json").write_text(json.dumps(job))
        cmd = ([node, str(C.RUN_JOB), str(rd / "job.json")] if build == "wasm-bench" else
               [str(REPO / "scripts" / "pyrun"), "-I", str(REPO / "scripts" / "browser_job.py"),
                str(rd / "job.json")])
        if build == "wasm-bench" and not node:
            return {"skip": "no Node"}
    elif build == "wasm-node":
        if not node or not C.NODE_PAIR.exists():
            return {"skip": "no Node or no build/s32pair_node.js"}
        cmd = [node, str(C.NODE_PAIR)] + args
    else:
        cmd = [str(C.PAIR)] + args
    r = subprocess.run(cmd, env=dict(os.environ, **env), cwd=rd, capture_output=True, text=True)
    shutil.rmtree(rd, ignore_errors=True)
    if r.returncode:
        return {"error": f"exit {r.returncode}: {(r.stdout + r.stderr)[-300:]}"}
    if build in ("wasm-bench", "browser"):
        j = json.loads(r.stdout.strip().splitlines()[-1])
        return {"x": j["x_real_time"], "wall": j["wall_s"], "browser": j.get("browser")}
    m = re.search(r"emulated ([0-9.]+) s in [0-9.]+ s wall \(([0-9.]+)x real time\)", r.stdout)
    if not m:
        return {"error": "no speed line in the output"}
    return {"x": float(m.group(2)), "wall": secs / float(m.group(2))}


def bench(manifest: list[dict], builds: list[str], work: pathlib.Path, fwfiles: dict[str, str], repeat: int) -> dict:
    out: dict = {}
    for e in manifest:
        for b in builds:
            runs = [run_speed(e, b, work, fwfiles) for _ in range(repeat)]
            good = [r for r in runs if "x" in r]
            if len(good) == len(runs):
                out[(e["id"], b)] = {"x": statistics.median(r["x"] for r in good), "secs": e["secs"],
                                     "all": [round(r["x"], 2) for r in good], "browser": good[0].get("browser")}
            else:
                bad = next(r for r in runs if "x" not in r)
                out[(e["id"], b)] = bad
            r = out[(e["id"], b)]
            print(f"  {b:14s} {e['id']:16s} " + (f"{r['x']:6.2f}x real time  (runs {r['all']})" if "x" in r else
                                               f"{'SKIP ' + r['skip'] if 'skip' in r else 'ERROR ' + r['error']}"), flush=True)
    return out


def table(title: str, mach: dict, res: dict, scen: list[str], builds: list[str], browser: str | None = None) -> str:
    lines = [f"**{title}** -- {mach.get('cpu', '?')} ({mach.get('cores', '?')} logical CPUs), {mach['os']}; "
             f"{mach['cc']}; native flags `{mach['native_flags']}`; {mach['emcc']} `{mach['wasm_flags']}`; "
             f"Node {mach['node']}" + (f"; headless Chromium {browser}" if browser else "") +
             f"; load average at start {mach['load']}", "",
             "| scenario | " + " | ".join(builds) + " |", "|---|" + "---|" * len(builds)]
    for s in scen:
        row = []
        for b in builds:
            r = res.get((s, b), {})
            row.append(f"{r['x']:.1f}x" if "x" in r else "skip" if "skip" in r else "error" if "error" in r else "-")
        lines.append(f"| {s.split('/', 1)[1]} | " + " | ".join(row) + " |")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--firmware-dir", type=pathlib.Path, default=None)
    ap.add_argument("--builds", default=",".join(BUILDS))
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--only", nargs="*", default=None, help="scenario names (prefix match)")
    ap.add_argument("--remote", default=None)
    ap.add_argument("--remote-dir", default="s32-conformance")
    ap.add_argument("--remote-env", default="")
    ap.add_argument("--remote-builds", default="native,native-serial,wasm-node,wasm-bench")
    ap.add_argument("--json", type=pathlib.Path, default=None)
    ap.add_argument("--worker", default=None, help=argparse.SUPPRESS)
    ap.add_argument("--worker-fw", default=None, help=argparse.SUPPRESS)
    a = ap.parse_args()
    builds = [b for b in a.builds.split(",") if b]
    if a.worker:
        work = pathlib.Path(a.worker)
        manifest = json.loads((work / "manifest.json").read_text())
        ct, bfl, pfl = a.worker_fw.split(",")
        res = bench(manifest, builds, work, {"ct": ct, "bfl": bfl, "pfl": pfl}, a.repeat)
        (work / "bench.json").write_text(json.dumps({"machine": machine(),
                                                     "results": [{"id": i, "build": b, **r} for (i, b), r in res.items()]}))
        return 0
    if a.firmware_dir is None and os.environ.get("S32_FIRMWARE_DIR"):
        a.firmware_dir = pathlib.Path(os.environ["S32_FIRMWARE_DIR"]).expanduser()
    if a.firmware_dir is None:
        print("bench: SKIP: no firmware (set S32_FIRMWARE_DIR or pass --firmware-dir; none ships with this repository)")
        return 0
    fw, why = C.K.find_firmware(a.firmware_dir.resolve())
    if fw is None:
        print(f"bench: FAIL: firmware directory unusable: {why}")
        return 1
    fwfiles = {k: str(fw[k]) for k in ("ct", "bfl", "pfl")}
    only = [f"bench/{o}" for o in a.only] if a.only else ["bench/"]
    cases = C.load_cases("fast", only, 5.0, bench_only=True)
    work = pathlib.Path(tempfile.mkdtemp(prefix="bench-"))
    manifest = [C.prepare_case(c, fw, work, {}) for c in cases]
    (work / "manifest.json").write_text(json.dumps(manifest))
    mach = machine()
    print(f"bench: {mach.get('cpu')} / {mach['cc']} / load {mach['load']}", flush=True)
    res = bench(manifest, builds, work, fwfiles, a.repeat)
    br = next((r.get("browser") for (i, b), r in res.items() if b == "browser" and r.get("browser")), None)
    mach["load_end"] = " ".join(f"{x:.1f}" for x in os.getloadavg())
    scen = [e["id"] for e in manifest]
    out = [table("This machine", mach, res, scen, builds, br)] if builds else []
    rem = None
    if a.remote:
        rdir = a.remote_dir
        if not rdir.startswith("/"):
            rdir = C.ssh(a.remote, "echo $HOME").stdout.strip() + "/" + rdir
        C.ssh(a.remote, f"mkdir -p {rdir}/src {rdir}/fw {rdir}/bench && chmod 700 {rdir} {rdir}/fw")
        C.rsync(["--exclude=.git", "--exclude=build/", "--exclude=__pycache__",
                 f"{REPO}/"], f"{a.remote}:{rdir}/src/")
        C.rsync([f"{REPO}/emu/wasm/build/"], f"{a.remote}:{rdir}/src/emu/wasm/build/",
                ["--include=*.js", "--include=*.wasm", "--exclude=*"])
        C.rsync([fwfiles["ct"], fwfiles["bfl"], fwfiles["pfl"]], f"{a.remote}:{rdir}/fw/")
        C.ssh(a.remote, f"chmod 600 {rdir}/fw/*")
        C.rsync([f"{work}/cases/"], f"{a.remote}:{rdir}/bench/cases/")
        C.rsync([str(work / "manifest.json")], f"{a.remote}:{rdir}/bench/")
        C.ssh(a.remote, f"cd {rdir}/src && {a.remote_env} nice -n 19 make -s emu >/dev/null")
        names = [pathlib.Path(fwfiles[k]).name for k in ("ct", "bfl", "pfl")]
        r = C.ssh(a.remote, f"cd {rdir}/src && {a.remote_env} python3 scripts/bench.py --worker {rdir}/bench --worker-fw "
                            f"{','.join(rdir + '/fw/' + n for n in names)} --builds {a.remote_builds} --repeat {a.repeat}",
                  check=False, capture=False)
        if r.returncode:
            print("bench: remote worker failed")
            return 1
        C.rsync([f"{a.remote}:{rdir}/bench/bench.json"], str(work / "remote_bench.json"))
        rem = json.loads((work / "remote_bench.json").read_text())
        rres = {(x["id"], x["build"]): x for x in rem["results"]}
        out.append(table("Second machine", rem["machine"], rres, scen, [b for b in a.remote_builds.split(",") if b]))
    print("\n" + "\n\n".join(out))
    if a.json:
        a.json.write_text(json.dumps({"machine": mach, "results": [{"id": i, "build": b, **r} for (i, b), r in res.items()],
                                  "remote": rem}, indent=1))
    shutil.rmtree(work, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
