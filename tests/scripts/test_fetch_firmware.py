# SPDX-License-Identifier: Apache-2.0
"""scripts/fetch_firmware.py on generated files (no firmware): the HEX and obfuscation decoders, the image layouts,
and that a size or SHA-256 difference is refused."""
from __future__ import annotations

import hashlib
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "scripts"))
import fetch_firmware as F  # noqa: E402


def rec(addr: int, typ: int, body: bytes) -> str:
    r = bytes([len(body), (addr >> 8) & 0xFF, addr & 0xFF, typ]) + body
    return ":" + (r + bytes([(-sum(r)) & 0xFF])).hex().upper()


def obfuscate(plain: bytes, key: int) -> bytes:
    out, k = bytearray(), key
    for p in plain:
        out.append((p + k) & 0xFF)
        k = (k - 1) & 0xFF
        if k == 0:
            k = key
    return bytes(out)


class Decoders(unittest.TestCase):
    def test_key_from_name(self):
        self.assertEqual(F.ec_key("EC086001.bin"), 86001 & 0xFF)

    def test_deobfuscate_inverts(self):
        plain = bytes(range(256)) * 5
        for key in (1, 7, 241, 255):
            self.assertEqual(F.ec_deobfuscate(obfuscate(plain, key), key), plain)

    def test_bad_checksum_refused(self):
        with self.assertRaises(F.Refused):
            F.parse_hex(":0100000000FE\n")

    def test_ct_prog_layout(self):
        # PC 0 = 0x040200, PC 2 = 0x000000 (byte addresses 0..7, phantom byte 0)
        text = "\n".join([rec(0, 0, bytes([0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00])), rec(0, 1, b"")])
        self.assertEqual(F.ct_prog(text.encode()), bytes([0x00, 0x02, 0x04, 0x00, 0x00, 0x00]))

    def test_ec_flash_layout(self):
        lines = [rec(0, 4, bytes([0x1D, 0x00])), rec(0x0010, 0, b"\x11\x22"),
                 rec(0, 4, bytes([0x1F, 0xC0])), rec(0x0000, 0, b"\x33"), rec(0, 1, b"")]
        bfl, pfl = F.ec_flash("EC086001.bin", obfuscate("\n".join(lines).encode(), F.ec_key("EC086001.bin")))
        self.assertEqual(pfl, b"\xff" * 16 + b"\x11\x22")
        self.assertEqual(len(bfl), F.PIC32_BFLASH_SIZE)
        self.assertEqual(bfl[:2], b"\x33\xff")


class Verify(unittest.TestCase):
    def test_local_mismatch_refused(self):
        with tempfile.TemporaryDirectory() as t:
            p = pathlib.Path(t) / "CT000001.bin"
            p.write_bytes(b"not the file")
            e = {"name": p.name, "size": 12, "sha256": "0" * 64}
            with self.assertRaises(F.Refused):
                F.obtain(e, pathlib.Path(t), "https://example.invalid/", pathlib.Path(t) / "dl")
            e["sha256"] = hashlib.sha256(b"not the file").hexdigest()
            self.assertEqual(F.obtain(e, pathlib.Path(t), "https://example.invalid/", pathlib.Path(t) / "dl"), b"not the file")

    def test_placeholder_url_refused_without_local_copy(self):
        with tempfile.TemporaryDirectory() as t:
            e = {"name": "CT000001.bin", "size": 1, "sha256": "0" * 64}
            with self.assertRaises(F.Refused):
                F.obtain(e, None, "https://github.com/OWNER/REPO/releases/download/x/", pathlib.Path(t))

    def test_manifest_is_complete(self):
        import json
        m = json.loads(F.MANIFEST.read_text())
        for e in m["files"] + m["derived"]:
            self.assertRegex(e["sha256"], "^[0-9a-f]{64}$")
            self.assertGreater(e["size"], 0)
        self.assertIn("LICENSE-FIRMWARE", m["license"])


if __name__ == "__main__":
    unittest.main()
