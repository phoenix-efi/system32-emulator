#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""FAT32 SD-card images for the emulator's card slots (emu/pair S32_CT_SD, S32_ECU1_SD, S32_ECU2_SD).

    scripts/pyrun -I scripts/sdcard_image.py new <img> [MB=64]
    scripts/pyrun -I scripts/sdcard_image.py put <img> <host-file> [/NAME]
    scripts/pyrun -I scripts/sdcard_image.py get <img> /NAME <host-file>
    scripts/pyrun -I scripts/sdcard_image.py ls  <img>
    scripts/pyrun -I scripts/sdcard_image.py mk  <img> <dir> [MB=64] [--alias LONG=SHORT ...]
                                                                        (new card holding every file in dir)
    scripts/pyrun -I scripts/sdcard_image.py dump <img> <dir>          (every root file to dir)

Per-file Python (pyfatfs), never inside an emulation loop. The emulated card is SDHC in SPI mode (emu/controller/src/
s32dev.c); any FAT32 image works, these commands only make preparing one convenient. `new` and `mk` write the same
bytes for the same files on every run (fixed volume ID and root-directory time stamps, fix_times).

--alias LONG=SHORT gives the file LONG the FAT short (8.3) name SHORT, the alias a host stores beside a long name
(e.g. "EC086001 (1).bin=EC0860~1.BIN", measured on a macOS FAT32 card). pyfatfs generates its own aliases, which differ
from the ones desktop hosts write; firmware that looks only at short names sees what the alias says.
"""
from __future__ import annotations

import os
import struct
import sys


def lfn_checksum(sfn: bytes) -> int:
    """The checksum every long-name entry carries of its 11-byte short name (FAT specification)."""
    c = 0
    for b in sfn:
        c = (((c & 1) << 7) + (c >> 1) + b) & 0xFF
    return c


def sfn_bytes(short: str) -> bytes:
    base, _, ext = short.upper().partition(".")
    if not (1 <= len(base) <= 8 and len(ext) <= 3) or any(ch in base + ext for ch in ' "*+,./:;<=>?[\\]|'):
        raise SystemExit(f"not a valid 8.3 name: {short}")
    return (base.ljust(8) + ext.ljust(3)).encode("ascii")


def set_short_name(img: str, long: str, short: str) -> None:
    """Rewrite the short-name entry of the root-directory file whose long name is `long` to `short`, and the
    checksum in its long-name entries. Refuses a short name another root entry already has."""
    want = sfn_bytes(short)
    with open(img, "r+b") as f:
        bs = f.read(512)
        bps, spc, rsvd, nfats = struct.unpack_from("<HBHB", bs, 11)
        fatsz, root = struct.unpack_from("<I", bs, 36)[0], struct.unpack_from("<I", bs, 44)[0]
        csize, data0 = bps * spc, (rsvd + nfats * fatsz) * bps
        entries: list[int] = []                                  # file offsets of every root-directory entry
        clus = root
        while 2 <= clus < 0x0FFFFFF8:
            entries += [data0 + (clus - 2) * csize + i for i in range(0, csize, 32)]
            f.seek(rsvd * bps + clus * 4)
            clus = struct.unpack("<I", f.read(4))[0] & 0x0FFFFFFF
        lfn: list[int] = []
        target = None
        sfns: set[bytes] = set()
        for off in entries:
            f.seek(off)
            e = f.read(32)
            if e[0] == 0x00:
                break
            if e[0] == 0xE5:
                lfn = []
                continue
            if e[11] == 0x0F:
                lfn.append(off)
                continue
            parts = []
            for lo in reversed(lfn):
                f.seek(lo)
                le = f.read(32)
                parts.append(le[1:11] + le[14:26] + le[28:32])
            name = b"".join(parts).decode("utf-16-le").split("\0")[0]
            if lfn and name == long:
                target = (off, list(lfn))
            elif e[11] & 0x08 == 0:
                sfns.add(e[:11])
            lfn = []
        if target is None:
            raise SystemExit(f"{img}: no root file with the long name {long!r}")
        if want in sfns:
            raise SystemExit(f"{img}: short name {short} is already in use")
        off, lfns = target
        f.seek(off)
        f.write(want)
        f.seek(off + 12)
        f.write(b"\0")                                          # no lower-case flags: the long name carries the case
        for lo in lfns:
            f.seek(lo + 13)
            f.write(bytes([lfn_checksum(want)]))


FIXED_VOLUME_ID = 0x50484E58                 # any constant: the same card bytes on every run
FIXED_DOS_DATE = ((2026 - 1980) << 9) | (1 << 5) | 1   # 2026-01-01
FIXED_DOS_TIME = 0                                    # 00:00:00


def fix_times(img: str) -> None:
    """Make a new card reproducible: pyfatfs stamps the volume serial number and every directory entry with the wall
    clock, so the same files gave different card bytes on every run (and different emulator traces). Set the volume
    ID (boot sector and its backup) and the root directory's time stamps to fixed values."""
    with open(img, "r+b") as f:
        bs = f.read(512)
        bps, spc, rsvd, nfats = struct.unpack_from("<HBHB", bs, 11)
        fatsz, root = struct.unpack_from("<I", bs, 36)[0], struct.unpack_from("<I", bs, 44)[0]
        bk = struct.unpack_from("<H", bs, 50)[0]
        for sec in {0, bk} if bk else {0}:
            f.seek(sec * bps + 67)
            f.write(struct.pack("<I", FIXED_VOLUME_ID))
        csize, data0 = bps * spc, (rsvd + nfats * fatsz) * bps
        clus = root
        while 2 <= clus < 0x0FFFFFF8:
            for off in range(data0 + (clus - 2) * csize, data0 + (clus - 1) * csize, 32):
                f.seek(off)
                e = f.read(32)
                if e[0] == 0x00:
                    return
                if e[0] == 0xE5 or e[11] == 0x0F:
                    continue
                f.seek(off + 13)
                f.write(struct.pack("<BHHH", 0, FIXED_DOS_TIME, FIXED_DOS_DATE, FIXED_DOS_DATE))   # create, access
                f.seek(off + 22)
                f.write(struct.pack("<HH", FIXED_DOS_TIME, FIXED_DOS_DATE))                         # write
            f.seek(rsvd * bps + clus * 4)
            clus = struct.unpack("<I", f.read(4))[0] & 0x0FFFFFFF


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__)
        return 2
    from pyfatfs.PyFat import PyFat
    from pyfatfs.PyFatFS import PyFatFS

    aliases = []
    while "--alias" in argv:
        i = argv.index("--alias")
        aliases.append(argv[i + 1].rsplit("=", 1))
        del argv[i:i + 2]
    order: list[str] = []
    while "--order" in argv:
        i = argv.index("--order")
        order += [x for x in argv[i + 1].split(",") if x]
        del argv[i:i + 2]
    cmd, img = argv[0], argv[1]
    if cmd in ("new", "mk"):
        mb = int(argv[3 if cmd == "mk" else 2]) if len(argv) > (3 if cmd == "mk" else 2) else 64
        with open(img, "wb") as f:
            f.truncate(mb * 1024 * 1024)
        pf = PyFat()
        pf.mkfs(img, PyFat.FAT_TYPE_FAT32, size=mb * 1024 * 1024, label="CARD")
        pf.close()
        if cmd == "mk":
            all_names = sorted(os.listdir(argv[2]))
            names = [n for n in order if n in all_names] + [n for n in all_names if n not in order]
            with PyFatFS(img) as v:
                for name in names:
                    with open(os.path.join(argv[2], name), "rb") as a, v.openbin("/" + name, "w") as b:
                        b.write(a.read())
            for long, short in aliases:
                set_short_name(img, long, short)
        fix_times(img)
        print(f"{img}: {mb} MB FAT32")
        return 0
    with PyFatFS(img) as v:
        if cmd == "put":
            src = argv[2]
            dst = argv[3] if len(argv) > 3 else "/" + os.path.basename(src).upper()
            with open(src, "rb") as a, v.openbin(dst, "w") as b:
                b.write(a.read())
            print("put", dst)
        elif cmd == "get":
            with v.openbin(argv[2], "r") as a, open(argv[3], "wb") as b:
                b.write(a.read())
            print("get", argv[2])
        elif cmd == "dump":
            os.makedirs(argv[2], exist_ok=True)
            for name in sorted(v.listdir("/")):
                if v.getinfo("/" + name).is_dir:
                    continue
                with v.openbin("/" + name, "r") as a, open(os.path.join(argv[2], name), "wb") as b:
                    b.write(a.read())
        elif cmd == "ls":
            for name in sorted(v.listdir("/")):
                print(f"{v.getinfo('/' + name, namespaces=['details']).size:10d}  {name}")
        else:
            print(__doc__)
            return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
