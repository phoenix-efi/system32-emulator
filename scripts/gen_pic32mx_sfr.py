#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Generate model/pic32mx_sfr.json -- PIC32MX SFR names, addresses, widths and bitfields -- from Microchip's
device family pack (DFP), for the ECU part candidates.

    python3 scripts/gen_pic32mx_sfr.py [--pack <local .atpack>] [-o model/pic32mx_sfr.json] [--check] [PART ...]

Source of truth: the public PIC32MX DFP, fetched by pinned URL and SHA-256 (scripts/dfp_fetch.py; never committed),
its EDC files (`edc/<PART>.PIC`, XML). Each `edc:SFRDef` with an `edc:_addr` (physical) is one SFR; its bitfields come
from the `DS.0` SFRMode (the datasheet view): fields are laid out from bit 0 upwards in document order,
`edc:AdjustPoint edc:offset=N` skips N bits. `edc:portals="CLR SET INV"` means the PIC32 atomic alias registers at
+4 / +8 / +0xC. Addresses are emitted as KSEG1 virtual (phys | 0xA0000000), the way software touches them.
`edc:InterruptList/edc:Interrupt` = vector numbers, `edc:InterruptRequest` = IRQ (IFSx/IECx bit) numbers.

Parts are MERGED: one entry per (name, address, layout) with the list of parts that have it, so the JSON stays small and
"which part" questions are a filter. --check regenerates in memory and fails if the committed file differs.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dfp_fetch import open_pack, pack_version  # noqa: E402

PACK = "Microchip.PIC32MX_DFP.1.7.380"
DEFAULT_PARTS = ["PIC32MX370F512H", "PIC32MX470F512H", "PIC32MX695F512H", "PIC32MX795F512H", "PIC32MX695F512L",
                 "PIC32MX795F512L"]

EDC = "{http://crownking/edc}"
PORTAL_OFFSETS = {"CLR": 4, "SET": 8, "INV": 12}


def _a(el: ET.Element, name: str, default=None):
    return el.get(EDC + name, default)


def _int(s: str) -> int:
    return int(s, 0)


def kseg1(phys: int) -> int:
    """Physical SFR/flash/RAM address -> KSEG1 (uncached) virtual address."""
    return (phys & 0x1FFFFFFF) | 0xA0000000


def parse_fields(sfrdef: ET.Element) -> list[list]:
    """Bitfields [name, lsb, width] from the DS.0 mode (or the first mode), laid out from bit 0 in document order."""
    modes = sfrdef.findall(f"{EDC}SFRModeList/{EDC}SFRMode")
    if not modes:
        return []
    mode = next((m for m in modes if _a(m, "id") == "DS.0"), modes[0])
    out, bit = [], 0
    for el in mode:
        tag = el.tag.replace(EDC, "")
        if tag == "AdjustPoint":
            bit += _int(_a(el, "offset", "0"))
        elif tag == "SFRFieldDef":
            w = _int(_a(el, "nzwidth", "1"))
            out.append([_a(el, "cname") or _a(el, "name"), bit, w])
            bit += w
    return out


def parse_edc(raw: bytes) -> dict:
    """One part: {'sfrs': [...], 'vectors': [[n, cname, desc]], 'irqs': [[n, cname]]}."""
    root = ET.fromstring(raw)
    sfrs = []
    for d in root.iter(EDC + "SFRDef"):
        addr = _a(d, "_addr")
        if addr is None:  # core (NMMR) registers: zero, at, ... -- no memory address
            continue
        portals = (_a(d, "portals") or "").split()
        por = _a(d, "por") or ""
        sfrs.append({
            "name": _a(d, "cname") or _a(d, "name"),
            "addr": kseg1(_int(addr)),
            "width": _int(_a(d, "nzwidth", "0x20")),
            "portals": [p for p in portals if p in PORTAL_OFFSETS],
            "fields": parse_fields(d),
            "por": f"0x{int(''.join('1' if c == '1' else '0' for c in por) or '0', 2):08X}",
            "grp": _a(d, "grp") or "",
        })
    vectors, irqs = [], []
    il = root.find(f".//{EDC}InterruptList")
    if il is not None:
        for v in il.findall(f"{EDC}Interrupt"):
            vectors.append([_int(_a(v, "irq")), _a(v, "cname"), _a(v, "desc") or ""])
        for r in il.findall(f"{EDC}InterruptRequest"):
            irqs.append([_int(_a(r, "irq")), _a(r, "cname")])
    vectors.sort()
    irqs.sort()
    return {"sfrs": sfrs, "vectors": vectors, "irqs": irqs}


def merge(parts: dict[str, dict]) -> list[dict]:
    """Merge per-part SFR lists into one list keyed by (name, addr, width, portals, fields) with a `parts` list."""
    merged: dict[tuple, dict] = {}
    for part, data in parts.items():
        for s in data["sfrs"]:
            key = (s["name"], s["addr"], s["width"], tuple(s["portals"]), json.dumps(s["fields"]), s["por"], s["grp"])
            if key not in merged:
                merged[key] = dict(s, parts=[])
            merged[key]["parts"].append(part)
    out = sorted(merged.values(), key=lambda s: (s["addr"], s["name"], s["parts"]))
    for s in out:
        s["addr"] = f"0x{s['addr']:08X}"
    return out


def build(z, part_names: list[str]) -> dict:
    parts, raws = {}, {}
    for p in part_names:
        name = f"edc/{p}.PIC"
        if name not in z.namelist():
            raise SystemExit(f"no EDC file for {p} in the pack")
        raws[p] = z.read(name)
        parts[p] = parse_edc(raws[p])
    version = pack_version(z)
    return {
        "_comment": "GENERATED by scripts/gen_pic32mx_sfr.py from the Microchip PIC32MX DFP EDC files (pack and "
                    "per-part sha256 below) -- do not hand-edit. addr = KSEG1 virtual; portals CLR/SET/INV at +4/+8/+0xC; fields = [name, lsb, width]; por = power-on reset value (unimplemented and undefined bits as 0); grp = the EDC peripheral group; "
                    "vectors = [vector, cname, desc]; irqs = [irq (IFS/IEC bit), cname].",
        "dfp": {"name": "Microchip.PIC32MX_DFP", "version": version,
                "edc_sha256": {p: hashlib.sha256(raws[p]).hexdigest()[:16] for p in part_names}},
        "parts": part_names,
        "vectors": {p: parts[p]["vectors"] for p in part_names},
        "irqs": {p: parts[p]["irqs"] for p in part_names},
        "sfrs": merge(parts),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("parts", nargs="*", default=DEFAULT_PARTS)
    ap.add_argument("--pack", type=Path, default=None, help="local copy of the pinned .atpack (else download/cache)")
    ap.add_argument("-o", "--out", type=Path, default=Path(__file__).resolve().parent.parent / "model" / "pic32mx_sfr.json")
    ap.add_argument("--check", action="store_true", help="fail if the committed file differs from a fresh generation")
    a = ap.parse_args(argv)
    doc = build(open_pack(PACK, a.pack), a.parts)
    text = json.dumps(doc, indent=0, separators=(",", ":")) + "\n"
    if a.check:
        same = a.out.is_file() and a.out.read_text() == text
        print(f"{a.out}: {'matches' if same else 'DIFFERS from'} the generator output", file=sys.stderr)
        return 0 if same else 1
    a.out.write_text(text)
    print(f"{a.out}: {len(doc['sfrs'])} SFR entries for {', '.join(a.parts)} (DFP {doc['dfp']['version']})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
