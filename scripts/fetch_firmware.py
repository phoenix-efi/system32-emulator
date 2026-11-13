#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Fetch, verify and unpack the default firmware (firmware/manifest.json) into a git-ignored cache.

    scripts/fetch_firmware.py [--cache DIR] [--src DIR] [--bench-dir DIR] [--no-derive]

The stock v86 update files are FlyEFII's copyright and are never committed (firmware/LICENSE-FIRMWARE). They come
from, in this order:

  1. --src DIR or the S32_V86_SRC environment variable: a local directory holding the update files under their own
     names (CT086001.bin, EC086001.bin), e.g. the copy you received from the vendor. Nothing is downloaded.
  2. the cache itself, when it already holds them with the manifest's SHA-256 (an earlier run);
  3. the release assets the manifest names (release.base_url + file name), downloaded with urllib.

Every file must have the manifest's size and SHA-256; any difference is refused (exit 1) and the downloaded file is
deleted. Then the emulator images are derived (Controller program memory, ECU boot and program flash, and the ECU's
default configuration chip, which the native emulator writes by running the firmware's first power-up on a blank chip),
each checked against the manifest too. The cache (default .firmware-cache/v86, git-ignored) is then a firmware
directory for the native front end and the conformance gate: S32_FIRMWARE_DIR=.firmware-cache/v86.

--bench-dir DIR also writes the browser bench's default firmware: the two update files and the manifest.json the page
fetches and checks (emu/wasm/web/s32_bench.js s32FetchDefaultFirmware). Prints the firmware directory on the last line.
Exit status: 0 ready, 1 refused or failed, 2 usage error. Standard library only.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile
import urllib.request

REPO = pathlib.Path(__file__).resolve().parent.parent
MANIFEST = REPO / "firmware" / "manifest.json"
PAIR = REPO / "emu" / "pair" / "build" / "s32pair_shim"
PIC32_PFLASH = 0x1D000000
PIC32_BFLASH = 0x1FC00000
PIC32_BFLASH_SIZE = 0x3000


class Refused(Exception):
    pass


def sha256(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


# ---- the update-file formats --------------------------------------------------------------------------------------
# An ECU update file is Intel HEX with each byte offset by a counting-down key. The key is recovered from the file
# itself: Intel HEX records have a known plain-text structure (':', hex digits, a checksum on every record), and the key
# is the one value under which every record is valid (emu/wasm/src/s32_wasm.c test_ec_key tries each value that way).
# For the stock files that value is the low byte of the file number, which ec_key reads from the name.

def ec_key(name: str) -> int:
    """ECvvvbbb.bin: the key is the low byte of the six-digit file number, read as one decimal number."""
    return int(name[2:8]) & 0xFF


def ec_deobfuscate(data: bytes, key: int) -> bytes:
    """plain = cipher - k per byte (mod 256), k counting down from the key K to 1 and wrapping: K, K-1, ..., 1, K, ..."""
    out = bytearray(len(data))
    k = key
    for i, c in enumerate(data):
        out[i] = (c - k) & 0xFF
        k = (k - 1) & 0xFF
        if k == 0:
            k = key
    return bytes(out)


def parse_hex(text: str) -> dict[int, int]:
    """Intel HEX (record types 0, 1, 2, 4) -> {byte address: value}; every checksum is checked."""
    mem: dict[int, int] = {}
    base = 0
    for n, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        if line[0] != ":":
            raise Refused(f"line {n}: not an Intel HEX record")
        rec = bytes.fromhex(line[1:])
        ln, addr, typ = rec[0], (rec[1] << 8) | rec[2], rec[3]
        if len(rec) != ln + 5 or sum(rec) & 0xFF:
            raise Refused(f"line {n}: bad record length or checksum")
        body = rec[4:4 + ln]
        if typ == 0:
            for i, b in enumerate(body):
                mem[base + addr + i] = b
        elif typ == 4:
            base = ((body[0] << 8) | body[1]) << 16
        elif typ == 2:
            base = ((body[0] << 8) | body[1]) << 4
        elif typ == 1:
            break
        else:
            raise Refused(f"line {n}: record type {typ}")
    return mem


def ct_prog(raw: bytes) -> bytes:
    """Controller program memory: HEX byte address 2*PC holds a 24-bit word in 4 bytes (the 4th is the phantom byte);
    the image holds 3 bytes per instruction word (PC step 2) from PC 0 to the highest programmed address."""
    mem = parse_hex(raw.decode("ascii"))
    top_pc = (max(mem) + 1) // 2
    out = bytearray()
    for pc in range(0, top_pc, 2):
        a = 2 * pc
        out += bytes((mem.get(a, 0xFF), mem.get(a + 1, 0xFF), mem.get(a + 2, 0xFF)))
    return bytes(out)


def ec_flash(name: str, raw: bytes) -> tuple[bytes, bytes]:
    """(boot flash, program flash) of an ECU update file, HEX addresses physical."""
    mem = parse_hex(ec_deobfuscate(raw, ec_key(name)).decode("ascii"))
    pf = [a for a in mem if PIC32_PFLASH <= a < PIC32_BFLASH]
    top = max(pf) + 1 - PIC32_PFLASH
    pfl = bytes(mem.get(PIC32_PFLASH + i, 0xFF) for i in range(top))
    bfl = bytes(mem.get(PIC32_BFLASH + i, 0xFF) for i in range(PIC32_BFLASH_SIZE))
    return bfl, pfl


# ---- fetch and check ----------------------------------------------------------------------------------------------

def check(entry: dict, data: bytes, what: str) -> None:
    if len(data) != entry["size"] or sha256(data) != entry["sha256"]:
        raise Refused(f"{what} {entry['name']}: size {len(data)}, SHA-256 {sha256(data)}; the manifest says "
                      f"{entry['size']}, {entry['sha256']}")


def cached(entry: dict, d: pathlib.Path) -> bytes | None:
    """The file from an earlier run, if it is still the manifest's."""
    p = d / entry["name"]
    if p.is_file():
        data = p.read_bytes()
        if len(data) == entry["size"] and sha256(data) == entry["sha256"]:
            return data
    return None


def obtain(entry: dict, src: pathlib.Path | None, base_url: str, dl: pathlib.Path) -> bytes:
    if src is not None:
        p = src / entry["name"]
        if not p.is_file():
            raise Refused(f"{p} not found (S32_V86_SRC / --src names a directory holding the update files)")
        data = p.read_bytes()
        check(entry, data, "local file")
        return data
    target = dl / entry["name"]
    if target.is_file():
        data = target.read_bytes()
        if len(data) == entry["size"] and sha256(data) == entry["sha256"]:
            return data
        target.unlink()
    if "OWNER/REPO" in base_url:
        raise Refused("the manifest's release URL is still a placeholder and no local copy was given: set S32_V86_SRC "
                      "to a directory holding " + entry["name"])
    url = base_url + entry["name"]
    dl.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix(target.suffix + ".part")
    try:
        with urllib.request.urlopen(url, timeout=60) as r, open(tmp, "wb") as f:
            shutil.copyfileobj(r, f)
        data = tmp.read_bytes()
        check(entry, data, "download")
    except Exception:
        tmp.unlink(missing_ok=True)
        raise
    tmp.replace(target)
    return data


def first_power_up_fram(cache: pathlib.Path) -> bytes:
    """The ECU configuration chip after the first power-up on a blank chip (2 s, no other input)."""
    if not PAIR.exists():
        raise Refused(f"{PAIR} not built (make emu): needed to write default_ecu.fram")
    ct, bfl, pfl = (sorted(cache.glob(g))[0] for g in ("*_prog.bin", "*_bflash.bin", "*_pflash.bin"))
    with tempfile.TemporaryDirectory(prefix="s32-fram-") as t:
        out = pathlib.Path(t) / "e1.fram"
        env = {k: v for k, v in os.environ.items() if not k.startswith("S32_")}
        env.update(S32_ECU1_FRAM_OUT=str(out))
        r = subprocess.run([str(PAIR), str(ct), str(bfl), str(pfl), "2"], env=env, cwd=t, capture_output=True, text=True)
        if r.returncode or not out.exists():
            raise Refused(f"emulator run for default_ecu.fram failed (exit {r.returncode}): {r.stderr[-300:]}")
        return out.read_bytes()


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cache", type=pathlib.Path, default=REPO / ".firmware-cache" / "v86")
    ap.add_argument("--src", type=pathlib.Path, default=None, help="local directory with the update files (S32_V86_SRC)")
    ap.add_argument("--bench-dir", type=pathlib.Path, default=None, help="also write the bench's default firmware here")
    ap.add_argument("--no-derive", action="store_true", help="only fetch and check the update files")
    a = ap.parse_args(argv)
    m = json.loads(MANIFEST.read_text())
    src = a.src or (pathlib.Path(os.environ["S32_V86_SRC"]).expanduser() if os.environ.get("S32_V86_SRC") else None)
    cache = a.cache.resolve()
    try:
        raw = {e["name"]: (None if src else cached(e, cache)) or obtain(e, src, m["release"]["base_url"],
                                                                        cache.parent / "downloads") for e in m["files"]}
        cache.mkdir(parents=True, exist_ok=True)
        os.chmod(cache, 0o700)
        for name, data in raw.items():
            (cache / name).write_bytes(data)
        print(f"fetch_firmware: {len(raw)} update files verified ({'local ' + str(src) if src else 'cache or release assets'})")
        if not a.no_derive:
            want = {d["name"]: d for d in m["derived"]}
            ct = next(n for n in raw if n.upper().startswith("CT"))
            ec = next(n for n in raw if n.upper().startswith("EC"))
            bfl, pfl = ec_flash(ec, raw[ec])
            for name, data in ((next(n for n in want if n.endswith("_prog.bin")), ct_prog(raw[ct])),
                               (next(n for n in want if n.endswith("_bflash.bin")), bfl),
                               (next(n for n in want if n.endswith("_pflash.bin")), pfl)):
                check(want[name], data, "derived")
                (cache / name).write_bytes(data)
            fram = cache / "default_ecu.fram"
            if not (fram.is_file() and sha256(fram.read_bytes()) == want["default_ecu.fram"]["sha256"]):
                data = first_power_up_fram(cache)
                check(want["default_ecu.fram"], data, "derived")
                fram.write_bytes(data)
            print(f"fetch_firmware: {len(want)} emulator images derived and verified")
        if a.bench_dir:
            bd = a.bench_dir
            bd.mkdir(parents=True, exist_ok=True)
            for name in raw:
                shutil.copy(cache / name, bd / name)
            (bd / "manifest.json").write_text(json.dumps({
                "version": m["version"], "source_commit": "", "label": m["bench"]["label"], "note": m["bench"]["note"],
                "copyright": m["copyright"], "license": m["license"],
                "files": [{"name": e["name"], "size": e["size"], "sha256": e["sha256"]} for e in m["files"]]},
                indent=1) + "\n")
            print(f"fetch_firmware: bench default firmware in {bd}")
    except Refused as e:
        print(f"fetch_firmware: refused: {e}", file=sys.stderr)
        return 1
    except OSError as e:
        print(f"fetch_firmware: FAILED: {e}", file=sys.stderr)
        return 1
    print(cache)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
