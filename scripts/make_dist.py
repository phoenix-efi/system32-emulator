#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""The versioned WebAssembly bundle that other sites embed (a guide site, for one):

    scripts/make_dist.py --version V [--out dist]

writes dist/system32-emulator-wasm-V.tar.gz holding s32_wasm.js, s32_wasm.wasm, index.html (the bench page),
cockpit.html (the cockpit page), s32_bench.js (the script both pages load), firmware-manifest.json
(firmware/manifest.json: names, sizes, SHA-256 and release URLs of the default firmware, which is NOT in the bundle) and
SHA256SUMS, plus the same SHA256SUMS next to the archive. The archive is reproducible: fixed member order, mtime 0,
owner 0. Any image-like file in the bundle (.bin .hex .flat .fram .img) fails the run.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import io
import pathlib
import sys
import tarfile

REPO = pathlib.Path(__file__).resolve().parent.parent
FILES = {"s32_wasm.js": REPO / "emu/wasm/build/s32_wasm.js", "s32_wasm.wasm": REPO / "emu/wasm/build/s32_wasm.wasm",
         "index.html": REPO / "emu/wasm/build/index.html", "cockpit.html": REPO / "emu/wasm/build/cockpit.html",
         "s32_bench.js": REPO / "emu/wasm/build/s32_bench.js", "firmware-manifest.json": REPO / "firmware/manifest.json"}
IMAGE_SUFFIXES = (".bin", ".hex", ".flat", ".fram", ".img", ".elf")


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--version", required=True)
    ap.add_argument("--out", type=pathlib.Path, default=REPO / "dist")
    a = ap.parse_args(argv)
    missing = [str(p) for p in FILES.values() if not p.is_file()]
    if missing:
        print(f"make_dist: missing {', '.join(missing)} (make wasm)", file=sys.stderr)
        return 1
    bad = [n for n in FILES if n.lower().endswith(IMAGE_SUFFIXES)]
    if bad:
        print(f"make_dist: image-like file in the bundle: {bad}", file=sys.stderr)
        return 1
    data = {n: p.read_bytes() for n, p in FILES.items()}
    sums = "".join(f"{hashlib.sha256(b).hexdigest()}  {n}\n" for n, b in sorted(data.items())).encode()
    data["SHA256SUMS"] = sums
    root = f"system32-emulator-wasm-{a.version}"
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w", format=tarfile.PAX_FORMAT) as tar:
        for n in sorted(data):
            ti = tarfile.TarInfo(f"{root}/{n}")
            ti.size, ti.mtime, ti.mode, ti.uid, ti.gid = len(data[n]), 0, 0o644, 0, 0
            tar.addfile(ti, io.BytesIO(data[n]))
    a.out.mkdir(parents=True, exist_ok=True)
    arc = a.out / f"{root}.tar.gz"
    with open(arc, "wb") as f, gzip.GzipFile(fileobj=f, mode="wb", mtime=0, filename="") as g:
        g.write(buf.getvalue())
    (a.out / f"{root}.SHA256SUMS").write_bytes(sums)
    print(f"make_dist: {arc} ({arc.stat().st_size} bytes), sha256 {hashlib.sha256(arc.read_bytes()).hexdigest()}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
