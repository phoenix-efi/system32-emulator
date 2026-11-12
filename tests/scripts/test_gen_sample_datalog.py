#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Tests for scripts/gen_sample_datalog.py (the browser bench's synthetic demo datalog)."""
from __future__ import annotations

import importlib.util
import pathlib
import unittest

REPO = pathlib.Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("gen_sample_datalog", REPO / "scripts" / "gen_sample_datalog.py")
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)


class SampleDatalog(unittest.TestCase):
    def test_committed_page_is_current(self):
        self.assertEqual(gen.main(["--check"]), 0)

    def test_rows_match_the_column_layout(self):
        self.assertEqual(len(gen.COLUMNS), 27)
        for i in range(gen.ROWS):
            f = gen.row(i)
            self.assertEqual(len(f), len(gen.COLUMNS))
            self.assertTrue(f[2].isdigit())                      # the page's parser requires a numeric RPM1

    def test_power_reduction_is_visible(self):
        climb, cruise = gen.row(0), gen.row(gen.ROWS - 1)
        self.assertGreater(int(climb[2]), int(cruise[2]) + 150)
        self.assertGreater(float(climb[4]), float(cruise[4]) + 3.0)
        self.assertGreater(float(climb[21]), float(cruise[21]))

    def test_render_is_idempotent(self):
        page = gen.PAGE.read_text()
        self.assertEqual(gen.render(gen.render(page)), gen.render(page))


if __name__ == "__main__":
    unittest.main()
