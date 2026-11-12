#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Generate model/pic24fj256da210_sfr.json from Microchip's device description (EDC) in the device family pack.

    python3 scripts/gen_pic24_sfr.py [--pack <local .atpack>] [-o model/pic24fj256da210_sfr.json] [--check]

Input: the public PIC24F-DA-GB_DFP pack, fetched by pinned URL and SHA-256 (scripts/dfp_fetch.py; never committed),
file edc/PIC24FJ256DA210.PIC (XML, namespace "crownking/edc"). Output is GENERATED -- never hand-edit it.
--check regenerates in memory and fails if the committed file differs.

Schema (version 1), consumed by the PIC24 emulator (emu/pic24/, emu/pic24/tools/gen_sfr_reset.py):
  device, generator, source{pack, pack_version, file, sha256}
  memory{program{reset, ivt, aivt, code, config}, data{sfr, gpr, eds_window, eds_ram}}   [begin, end) byte/PC addrs
  sfrs[]      sorted by addr: {name, addr, width (bits), access (MSB..LSB, '-' = unimplemented), impl_mask,
               por (reset value, unimplemented bits 0), mclr, aliases[], fields[{name, bit, width}],
               alt_fields{mode_id: [...]}, joined (name of the 32/24-bit JoinedSFRDef it belongs to, if any)}
  joined[]    {name, addr, width, parts[]}
  traps[]     {name, index, desc, vector, alt_vector}      vector = 0x04 + 2*index (PC units)
  interrupts[] {name, irq, desc, vector, alt_vector}       vector = 0x14 + 2*irq, alt = vector + 0x100

Field bit positions: an SFRMode lists its fields from bit 0 upward; <AdjustPoint offset=n/> skips n bits. Mode
"DS.0" is the datasheet view (multi-bit fields) and becomes `fields`; "LT.*" modes (one entry per bit, the legacy
per-bit names) are dropped; any other mode goes to `alt_fields`.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dfp_fetch import open_pack, pack_version as dfp_pack_version  # noqa: E402

PACK = "Microchip.PIC24F-DA-GB_DFP.1.5.369"

NS = "{http://crownking/edc}"
DEVICE = "PIC24FJ256DA210"
SCHEMA_VERSION = 1


def a(el: ET.Element, key: str, default: str | None = None) -> str | None:
    return el.get(NS + key, default)


def num(s: str) -> int:
    return int(s, 0)


def bits_value(pattern: str) -> int:
    """'------0000000001' (MSB first) -> int; '-' and 'x'/'u' count as 0."""
    v = 0
    for ch in pattern:
        v = (v << 1) | (1 if ch == "1" else 0)
    return v


def impl_mask(access: str) -> int:
    m = 0
    for ch in access:
        m = (m << 1) | (0 if ch == "-" else 1)
    return m


def mode_fields(mode: ET.Element) -> list[dict]:
    out, pos = [], 0
    for ch in mode:
        tag = ch.tag.removeprefix(NS)
        if tag == "AdjustPoint":
            pos += num(a(ch, "offset"))
        elif tag == "SFRFieldDef":
            w = num(a(ch, "nzwidth"))
            out.append({"name": a(ch, "cname"), "bit": pos, "width": w})
            pos += w
    return out


def parse_sfr(el: ET.Element, joined: str | None = None) -> dict:
    access = a(el, "access", "")
    d = {
        "name": a(el, "cname"),
        "addr": num(a(el, "_addr")),
        "width": num(a(el, "nzwidth")),
        "access": access,
        "impl_mask": impl_mask(access),
        "por": bits_value(a(el, "por", "")),
        "mclr": bits_value(a(el, "mclr", "")),
        "aliases": [],
        "fields": [],
        "alt_fields": {},
    }
    if joined:
        d["joined"] = joined
    for al in el.iter(NS + "LegacyAlias"):
        d["aliases"].append(a(al, "cname"))
    ml = el.find(NS + "SFRModeList")
    if ml is not None:
        for mode in ml.findall(NS + "SFRMode"):
            mid = a(mode, "id")
            if mid.startswith("LT."):
                continue
            fl = mode_fields(mode)
            if mid == "DS.0":
                d["fields"] = fl
            elif fl:
                d["alt_fields"][mid] = fl
    if not d["alt_fields"]:
        del d["alt_fields"]
    return d


def generate(z) -> dict:
    raw = z.read(f"edc/{DEVICE}.PIC")
    root = ET.fromstring(raw)
    pack_version = dfp_pack_version(z)

    sfrs, joined = [], []
    for ds in root.iter(NS + "SFRDataSector"):
        for el in ds:
            tag = el.tag.removeprefix(NS)
            if tag == "SFRDef":
                sfrs.append(parse_sfr(el))
            elif tag == "JoinedSFRDef":
                jn = a(el, "cname")
                parts = [parse_sfr(p, jn) for p in el.findall(NS + "SFRDef")]
                sfrs.extend(parts)
                joined.append({"name": jn, "addr": num(a(el, "_addr")), "width": num(a(el, "nzwidth")),
                               "parts": [p["name"] for p in parts]})
    sfrs.sort(key=lambda s: (s["addr"], s["name"]))

    traps, ints = [], []
    il = root.find(NS + "InterruptList")
    ti = 0
    for el in il:
        tag = el.tag.removeprefix(NS)
        if tag == "Trap":
            v = 0x04 + 2 * ti
            traps.append({"name": a(el, "cname"), "index": ti, "desc": a(el, "desc"), "vector": v,
                          "alt_vector": v + 0x100})
            ti += 1
        elif tag == "Interrupt":
            irq = num(a(el, "irq"))
            v = 0x14 + 2 * irq
            ints.append({"name": a(el, "cname"), "irq": irq, "desc": a(el, "desc"), "vector": v,
                         "alt_vector": v + 0x100})
    ints.sort(key=lambda i: i["irq"])

    def rng(tag: str, regionid: str | None = None) -> list[int] | None:
        for el in root.iter(NS + tag):
            if regionid is None or a(el, "regionid") == regionid:
                return [num(a(el, "beginaddr")), num(a(el, "endaddr"))]
        return None

    memory = {
        "program": {"reset": rng("ResetSector"), "ivt": rng("VectorSector"), "aivt": rng("AltVectorSector"),
                    "code": rng("CodeSector", "program"), "config": rng("WORMHoleSector")},
        "data": {"sfr": rng("SFRDataSector"), "gpr": rng("GPRDataSector"), "eds_window": rng("EDSWindowSector"),
                 "eds_ram": rng("ExtendedDataSpace")},
    }
    return {
        "_comment": "GENERATED by scripts/gen_pic24_sfr.py from Microchip's EDC device file (pack, version and sha256 "
                    "in source below) -- do not hand-edit. Addresses: data-space bytes (SFRs) / program PC units "
                    "(vectors). Everything here is DOCUMENTED (vendor device file).",
        "schema_version": SCHEMA_VERSION,
        "device": DEVICE,
        "generator": "scripts/gen_pic24_sfr.py",
        "source": {"pack": "Microchip.PIC24F-DA-GB_DFP", "pack_version": pack_version,
                   "file": f"edc/{DEVICE}.PIC", "sha256": hashlib.sha256(raw).hexdigest()},
        "memory": memory,
        "sfrs": sfrs,
        "joined": joined,
        "traps": traps,
        "interrupts": ints,
    }


def dumps(model: dict) -> str:
    """Stable, diff-friendly JSON: one SFR / vector per line."""
    lines = ["{"]
    keys = list(model)
    for k in keys:
        v = model[k]
        comma = "," if k != keys[-1] else ""
        if isinstance(v, list) and v and isinstance(v[0], dict):
            lines.append(f' "{k}": [')
            for i, item in enumerate(v):
                lines.append("  " + json.dumps(item, separators=(",", ":")) + ("," if i < len(v) - 1 else ""))
            lines.append(" ]" + comma)
        else:
            lines.append(f" {json.dumps(k)}: {json.dumps(v)}{comma}")
    lines.append("}")
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--pack", type=Path, default=None, help="local copy of the pinned .atpack (else download/cache)")
    ap.add_argument("-o", "--out", type=Path, default=Path(__file__).resolve().parent.parent / "model" / "pic24fj256da210_sfr.json")
    ap.add_argument("--check", action="store_true", help="fail if the committed file differs from a fresh generation")
    args = ap.parse_args(argv)
    model = generate(open_pack(PACK, args.pack))
    text = dumps(model)
    if args.check:
        same = args.out.is_file() and args.out.read_text() == text
        print(f"{args.out}: {'matches' if same else 'DIFFERS from'} the generator output", file=sys.stderr)
        return 0 if same else 1
    args.out.write_text(text)
    print(f"{args.out}: {len(model['sfrs'])} SFRs, {len(model['interrupts'])} interrupts, "
          f"{len(model['traps'])} traps", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
