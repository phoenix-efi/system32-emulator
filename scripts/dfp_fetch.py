#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Fetch a Microchip device family pack (DFP) by pinned URL and SHA-256, for the SFR model generators.

The packs are Microchip's public downloads (packs.download.microchip.com). They are never committed to this
repository: the generators read the few files they need straight out of the `.atpack` (a zip archive), from a local
cache (default ~/.cache/system32-emulator/dfp, or $S32_DFP_CACHE) or from a copy you pass with --pack. Every pack is
checked against the pinned SHA-256 before use; a mismatch is an error.
"""
from __future__ import annotations

import hashlib
import os
import urllib.request
import zipfile
from pathlib import Path

PACKS = {
    "Microchip.PIC24F-DA-GB_DFP.1.5.369": (
        "https://packs.download.microchip.com/Microchip.PIC24F-DA-GB_DFP.1.5.369.atpack",
        "94d9ba382bb257dc07fc4942868938a7791444a573f173cfae03a3642d84afed",
    ),
    "Microchip.PIC32MX_DFP.1.7.380": (
        "https://packs.download.microchip.com/Microchip.PIC32MX_DFP.1.7.380.atpack",
        "bf7e8ad4b707efb98eb3b142954405e4dc389af3fc73c67ec05e817a0ae79d04",
    ),
}


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def open_pack(name: str, local: Path | None = None, cache: Path | None = None) -> zipfile.ZipFile:
    """The pinned pack `name` as an open zip archive (downloaded once into the cache unless `local` is given)."""
    url, want = PACKS[name]
    if local is None:
        cache = cache or Path(os.environ.get("S32_DFP_CACHE", Path.home() / ".cache" / "system32-emulator" / "dfp"))
        cache.mkdir(parents=True, exist_ok=True)
        local = cache / f"{name}.atpack"
        if not local.is_file() or _sha256(local) != want:
            tmp = local.with_suffix(".part")
            with urllib.request.urlopen(url, timeout=120) as r, open(tmp, "wb") as f:
                while block := r.read(1 << 20):
                    f.write(block)
            tmp.replace(local)
    got = _sha256(local)
    if got != want:
        raise SystemExit(f"{local}: sha256 {got} does not match the pinned {want} for {name}")
    return zipfile.ZipFile(local)


def pack_version(z: zipfile.ZipFile) -> str | None:
    """The release version in the pack's .pdsc (the newest <release>)."""
    import xml.etree.ElementTree as ET

    pdsc = next((n for n in z.namelist() if n.endswith(".pdsc") and "/" not in n), None)
    if pdsc is None:
        return None
    rel = ET.fromstring(z.read(pdsc)).find(".//releases/release")
    return rel.get("version") if rel is not None else None
