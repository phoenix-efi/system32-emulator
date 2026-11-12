# SPDX-License-Identifier: Apache-2.0
"""scripts/s32case.py: the emulator's own case files load, their dependencies resolve, and no case judges firmware
behaviour (the case format has no checks)."""
from __future__ import annotations

import json
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "scripts"))
import s32case as K  # noqa: E402


class Cases(unittest.TestCase):
    def test_load_and_deps(self):
        cases = K.load_cases("full", None, None)
        ids = {c["id"] for c in cases}
        self.assertIn("emu/first-power-up", ids)
        self.assertIn("bench/boot", ids)
        for c in cases:
            for d in c["deps"]:
                self.assertIn(d, ids)
            self.assertNotIn("checks", c["case"])

    def test_only_presets_the_firmware_set_provides(self):
        for path, key in ((K.CASES, "cases"), (K.SCENARIOS, "scenarios")):
            for c in json.loads(path.read_text())[key]:
                self.assertIn(c["stimulus"].get("preset", "default"), K.PRESETS, c["name"])

    def test_home_sub(self):
        self.assertEqual(K.home_sub({"secs": "{H+2}"}, 5.0), {"secs": 7.0})
        with self.assertRaises(ValueError):
            K.home_time("{H-1}", 5.0)


if __name__ == "__main__":
    unittest.main()
