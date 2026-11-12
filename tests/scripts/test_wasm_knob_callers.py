#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Every JavaScript caller of the bench's s32_wasm_knob_names() handles its NULL return. The function returns NULL
when it cannot allocate the list (s32_wasm_knob_error() then says why); a caller that passes the pointer straight to
UTF8ToString reads address 0 and hands JSON.parse an empty string, so the real reason is lost. Each call must keep the
pointer and test it before reading it. Standard library only."""
from __future__ import annotations

import pathlib
import re
import subprocess
import unittest

REPO = pathlib.Path(__file__).resolve().parents[2]
CALL = re.compile(r"\b\w+\._s32_wasm_knob_names\(\)")
CHECKED = re.compile(r"const (\w+) = \w+\._s32_wasm_knob_names\(\);\s*\n\s*if \(!(\w+)\) ")


def callers() -> list[pathlib.Path]:
    r = subprocess.run(["git", "-C", str(REPO), "ls-files", "-z", "-co", "--exclude-standard", "--", "*.mjs", "*.js",
                        "*.html"], capture_output=True, text=True, check=True)
    files = [REPO / x for x in r.stdout.split("\0") if x]
    return [f for f in files if f.is_file() and CALL.search(f.read_text(errors="replace"))]


class WasmKnobCallers(unittest.TestCase):
    def test_callers_found(self):
        names = {str(f.relative_to(REPO)) for f in callers()}
        self.assertTrue({"emu/wasm/test/smoke.mjs", "emu/wasm/test/run_job.mjs", "emu/wasm/test/job_page.html"} <= names,
                        sorted(names))

    def test_every_call_checks_null_before_reading(self):
        for f in callers():
            text = f.read_text(errors="replace")
            calls = len(CALL.findall(text))
            checked = [m for m in CHECKED.finditer(text) if m[1] == m[2]]
            self.assertEqual(len(checked), calls,
                             f"{f.relative_to(REPO)}: {calls} call(s) of _s32_wasm_knob_names(), {len(checked)} kept in a "
                             f"variable and tested for NULL on the next line")


if __name__ == "__main__":
    unittest.main()
