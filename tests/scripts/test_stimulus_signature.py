#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Boot-snapshot stimulus signature (S32_STIMULUS; emu/common/s32board.c item_*, emu/pair/s32pair.c stimulus_take).

The boot-once planner lets one case restore another case's snapshot only when the two runs' static parts and
their timed items agree up to the snapshot point (scripts/s32case.py agree_until; the emulator re-checks with
s32snap_items_equal_before before any restore). A stimulus field left out of the timed items breaks that: two cases
differing only in that field would share a snapshot, and the restored run would replay the sibling case's stimulus.

The C unit test emu/pair/test/test_board.c checks the same signature one struct field at a time; the checks here run
the real binary (S32_STIMULUS) and derive the field lists from the source structs so the guard cannot go stale.
"""
from __future__ import annotations

import json
import math
import os
import pathlib
import re
import subprocess
import sys
import tempfile
import unittest

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts"))

import conformance  # noqa: E402
import s32case  # noqa: E402  (the snapshot helpers)

PAIR = REPO / "emu" / "pair" / "build" / "s32pair_shim"

# Stimulus item kinds (s32snap_item; s32snap.c KIND): 0 event, 1 analogue fault, 2 pin fault, 3 crank fault,
# 4 link fault, 5 engine start, 6 statistics window.
KIND_LINK_FAULT = 4


def stimulus(knobs: dict, fw: dict, d: pathlib.Path) -> dict:
    """What a run with these knobs sets up: the emulator's S32_STIMULUS report (the static part's hash and every
    timed item). The environment is scrubbed of S32_* first so an outer setting cannot leak into a run."""
    out = d / "stimulus.json"
    env = {k: v for k, v in os.environ.items() if not k.startswith("S32_")}
    r = subprocess.run([str(PAIR), str(fw["ct"]), str(fw["bfl"]), str(fw["pfl"]), "6"],
                       env={**env, **knobs, "S32_STIMULUS": str(out)}, cwd=d, capture_output=True, text=True)
    if r.returncode or not out.exists():
        raise AssertionError(f"stimulus run failed (exit {r.returncode}) for {knobs!r}\n{r.stderr}")
    return json.loads(out.read_text())


@unittest.skipUnless(PAIR.is_file(), "s32pair_shim not built")
class TestLinkFaultSignature(unittest.TestCase):
    """Two stimuli differing only in a link fault's arg3 (padded frame length) or cfg28 (a cfg-block
    word) must produce different timed items, so the planner never restores the sibling case's snapshot."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.tmp = tempfile.TemporaryDirectory()
        cls.fw = conformance.synthetic_firmware(pathlib.Path(cls.tmp.name) / "fw")

    @classmethod
    def tearDownClass(cls) -> None:
        cls.tmp.cleanup()

    def stim(self, knobs: dict) -> dict:
        return stimulus(knobs, self.fw, pathlib.Path(self.tmp.name))

    def check_pair(self, knob_a: str, knob_b: str) -> None:
        a = self.stim({"S32_ECU_LINK": knob_a})
        b = self.stim({"S32_ECU_LINK": knob_b})
        self.assertEqual(a["static"], b["static"])          # the difference is stimulus-only
        self.assertEqual(len(a["items"]), len(b["items"]))
        differing = [i for i, (x, y) in enumerate(zip(a["items"], b["items"])) if x != y]
        self.assertEqual(len(differing), 1, "exactly the link-fault item must differ")
        self.assertEqual(a["items"][differing[0]][0], KIND_LINK_FAULT)
        self.assertLess(s32case.agree_until(a["items"], b["items"]), math.inf)   # no shared snapshot past the fault

    def test_link_fault_arg3_changes_the_signature(self) -> None:
        self.check_pair("1:c2e:cfg-pad:5=100:100@2", "1:c2e:cfg-pad:5=100:200@2")

    def test_link_fault_cfg28_changes_the_signature(self) -> None:
        self.check_pair("2:e2c:cfg-block:1=10+2=20@2", "2:e2c:cfg-block:1=10+2=21@2")

    def test_identical_knobs_agree(self) -> None:
        """Control: the compare is not simply always-different."""
        a = self.stim({"S32_ECU_LINK": "1:c2e:cfg-pad:5=100:100@2"})
        b = self.stim({"S32_ECU_LINK": "1:c2e:cfg-pad:5=100:100@2"})
        self.assertEqual(a["static"], b["static"])
        self.assertEqual(a["items"], b["items"])
        self.assertEqual(s32case.agree_until(a["items"], b["items"]), math.inf)


# ------------------------------------------------------------------ completeness
# Every field of every stimulus item kind, proven to change the timed items. KNOB_PAIRS changes the field through the
# knobs and runs the real binary here; C_ONLY fields no knob pair isolates (runtime state the parser never sets, or a
# field no two knob values vary alone) -- test_board.c perturbs those one struct field at a time. NEVER_HASHED is the
# one field deliberately out of the hash. A field added to a struct lands in none of these and fails the source
# checks below, naming it.
KNOB_PAIRS = {
    "sb_afault": {  # S32_AN_FAULT who:ch=mode@t0[-t1]
        "t0": ({"S32_AN_FAULT": "1:5=gnd@1-2"}, {"S32_AN_FAULT": "1:5=gnd@1.5-2"}),
        "t1": ({"S32_AN_FAULT": "1:5=gnd@1-2"}, {"S32_AN_FAULT": "1:5=gnd@1-3"}),
        "period": ({"S32_AN_FAULT": "1:5=intermittent:20@1"}, {"S32_AN_FAULT": "1:5=intermittent:40@1"}),
        "val": ({"S32_AN_FAULT": "1:5=raw:300@1"}, {"S32_AN_FAULT": "1:5=raw:301@1"}),
        "noise": ({"S32_AN_FAULT": "1:5=noise:20@1"}, {"S32_AN_FAULT": "1:5=noise:21@1"}),
        "mode": ({"S32_AN_FAULT": "1:5=gnd@1"}, {"S32_AN_FAULT": "1:5=open@1"}),
    },
    "sb_pfault": {  # S32_PIN_FAULT who:RXn=mode@t0[-t1]  (port/bit: the fault claims the pin, so the static part
        # moves with them -- own/lvl are static configuration, not stimulus; the timed item still differs alone)
        "t0": ({"S32_PIN_FAULT": "1:RC13=gnd@1"}, {"S32_PIN_FAULT": "1:RC13=gnd@1.5"}),
        "t1": ({"S32_PIN_FAULT": "1:RC13=gnd@1-2"}, {"S32_PIN_FAULT": "1:RC13=gnd@1-3"}),
        "period": ({"S32_PIN_FAULT": "1:RC13=intermittent:20@1"}, {"S32_PIN_FAULT": "1:RC13=intermittent:40@1"}),
        "who": ({"S32_PIN_FAULT": "1:RC13=gnd@1"}, {"S32_PIN_FAULT": "2:RC13=gnd@1"}),
        "port": ({"S32_PIN_FAULT": "1:RC13=gnd@1"}, {"S32_PIN_FAULT": "1:RD13=gnd@1"}, False),
        "bit": ({"S32_PIN_FAULT": "1:RC13=gnd@1"}, {"S32_PIN_FAULT": "1:RC14=gnd@1"}, False),
        "mode": ({"S32_PIN_FAULT": "1:RC13=gnd@1"}, {"S32_PIN_FAULT": "1:RC13=open@1"}),
    },
    "sb_cfault": {
        "t0": ({"S32_CRANK_GLITCH": "1:2.5"}, {"S32_CRANK_GLITCH": "1:2.6"}),
        "t1": ({"S32_CRANK_DROP": "1:2-3"}, {"S32_CRANK_DROP": "1:2-4"}),
        "us": ({"S32_CRANK_GLITCH": "1:2.5:20"}, {"S32_CRANK_GLITCH": "1:2.5:40"}),
        "rate": ({"S32_CRANK_EMI": "2:100@1-2"}, {"S32_CRANK_EMI": "2:200@1-2"}),
        "k": ({"S32_CRANK_GLITCH": "1:2.5"}, {"S32_CRANK_GLITCH": "2:2.5"}),
        "kind": ({"S32_HALL_STUCK": "1:crank=low@2-3"}, {"S32_CRANK_DROP": "1:2-3:crank"}),
        "elem": ({"S32_CRANK_GLITCH": "1:2.5:20:crank"}, {"S32_CRANK_GLITCH": "1:2.5:20:sync"}),
    },
    "sb_lfault": {  # S32_ECU_LINK k:what[=v]@t0[-t1][/dt]
        "t0": ({"S32_ECU_LINK": "1:dead@1"}, {"S32_ECU_LINK": "1:dead@1.5"}),
        "t1": ({"S32_ECU_LINK": "1:dead@1-2"}, {"S32_ECU_LINK": "1:dead@1-3"}),
        "v": ({"S32_ECU_LINK": "1:ber=0.01@1"}, {"S32_ECU_LINK": "1:ber=0.02@1"}),
        "dt": ({"S32_ECU_LINK": "1:ber=0.01@1-2/0.1"}, {"S32_ECU_LINK": "1:ber=0.01@1-2/0.2"}),
        "n": ({"S32_ECU_LINK": "1:garbage=100@1"}, {"S32_ECU_LINK": "1:garbage=200@1"}),
        "arg1": ({"S32_ECU_LINK": "1:c2e:cfg:5=100@2"}, {"S32_ECU_LINK": "1:c2e:cfg:6=100@2"}),
        "arg2": ({"S32_ECU_LINK": "1:c2e:cfg:5=100@2"}, {"S32_ECU_LINK": "1:c2e:cfg:5=101@2"}),
        "arg3": ({"S32_ECU_LINK": "1:c2e:cfg-pad:5=100:100@2"}, {"S32_ECU_LINK": "1:c2e:cfg-pad:5=100:200@2"}),
        "cfg28": ({"S32_ECU_LINK": "2:e2c:cfg-block:1=10+2=20@2"}, {"S32_ECU_LINK": "2:e2c:cfg-block:1=10+2=21@2"}),
        "k": ({"S32_ECU_LINK": "1:dead@1"}, {"S32_ECU_LINK": "2:dead@1"}),
        "kind": ({"S32_ECU_LINK": "1:dead@1"}, {"S32_ECU_LINK": "1:cut-c2e@1"}),
        "sub": ({"S32_ECU_LINK": "1:c2e:status:860@2"}, {"S32_ECU_LINK": "1:c2e:status-short:860@2"}),
        "pace": ({"S32_ECU_LINK": "1:c2e:status:860@2"}, {"S32_ECU_LINK": "1:c2e:status:860:full@2"}),
        "burst": ({"S32_ECU_LINK": "1:c2e:status:860*2@2"}, {"S32_ECU_LINK": "1:c2e:status:860*3@2"}),
    },
    "sb_ev": {
        "t": ({"S32_ECU_POWER": "1:off@0.5"}, {"S32_ECU_POWER": "1:off@0.6"}),
        "a": ({"S32_ECU_POWER": "1:off@0.5"}, {"S32_ECU_POWER": "2:off@0.5"}),
        "v": ({"S32_SUPPLY": "1:12@0.5"}, {"S32_SUPPLY": "1:13@0.5"}),
        "s": ({"S32_CONSOLE_IN": "ct:1:0.5:foo"}, {"S32_CONSOLE_IN": "ct:1:0.5:bar"}),   # the text s selects
        "kind": ({"S32_INPUT": "0.5:snap:x"}, {"S32_CONSOLE_IN": "ct:1:0.5:x"}),         # kind (and a: the UART)
        "who": ({"S32_RESET": "ct@0.5"}, {"S32_RESET": "1@0.5"}),
        "bit": ({"S32_INPUT": "0.5:menu"}, {"S32_INPUT": "0.5:tune"}),                   # press and release items
        "level": ({"S32_ECU_POWER": "1:off@0.5"}, {"S32_ECU_POWER": "1:on@0.5"}),
    },
}
C_ONLY = {
    "sb_afault": {"frozen", "latched"},                # runtime state of a stuck fault; the parser never sets them
    "sb_pfault": {"us", "latched", "frozen"},          # us: no pin-fault mode sets it; latched/frozen: runtime
    "sb_cfault": set(),
    "sb_lfault": {"next_t", "done"},                   # next_t = t0 at parse; done: set when the fault fires
    "sb_ev": {"port"},                                 # every knob pair that moves port moves kind or bit with it
}
NEVER_HASHED = {
    # dyn is 1 only for events the running system added (ev_add: b->q > 0), so it is 0 in every parsed stimulus, and
    # the restore merge treats such events as state, not stimulus. test_board.c pins that perturbing it changes
    # nothing; if it ever should be hashed, move it into KNOB_PAIRS/C_ONLY and update item_ev.
    "sb_ev": {"dyn"},
}
STRUCTS = ("sb_afault", "sb_pfault", "sb_cfault", "sb_lfault", "sb_ev")
ITEM_FN = {"sb_afault": "item_af", "sb_pfault": "item_pf", "sb_cfault": "item_cf", "sb_lfault": "item_lf",
           "sb_ev": "item_ev"}
# The remaining timed items (s32board_stimulus, s32pair.c stimulus_take): the engine start and the statistics window.
EXTRA_PAIRS = {
    "engine.rpm": ({"S32_RPM": "1800"}, {"S32_RPM": "1801"}),
    "engine.start_s": ({"S32_START": "0.1"}, {"S32_START": "0.2"}),
    "stats.t": ({"S32_STATS_FROM": "0"}, {"S32_STATS_FROM": "1"}),
}
# Every knob that alters the run must move the signature (the static part or the timed items): a knob that changed
# nothing the signature sees could be restored away by a sibling case's snapshot. Non-default samples (the defaults
# leave the signature unchanged by definition). A knob may name a base environment both runs share (S32_ECU_SD_WP:
# the write-protect tab is state only with a card in the slot, so both runs attach an ECU1 SD image).
KNOB_SAMPLES = {
    "S32_RPM": "2400", "S32_START": "1", "S32_RPM_AT": "2=150,4=0", "S32_CYL": "6", "S32_SYNC_BEFORE": "25",
    "S32_MAG1": "70", "S32_MAG2": "77", "S32_HALL_WIDTH": "7", "S32_CRANK_JITTER": "10",
    "S32_CRANK_GLITCH": "1:2.5,2:3:20:sync", "S32_CRANK_DROP": "1:2-3:all", "S32_CRANK_EMI": "2:100@1-2",
    "S32_HALL_STUCK": "1:sync=low@1-2", "S32_AN": "5=900,1:9=600", "S32_CT_AN": "15=300", "S32_TPS_NOISE": "4",
    "S32_TPS_FAULT": "2:512~400@6", "S32_AN_FAULT": "ct:12=intermittent:20@1-2", "S32_TPS_SUPPLY": "2",
    "S32_TPS_UNPOWERED": "float", "S32_ET_WIRING": "both", "S32_BATT": "1=12.5,2=13@4", "S32_AFR": "14.7",
    "S32_SWITCHES": "on plead", "S32_ECU_SELECT": "2", "S32_SELECT_GAP_MS": "30", "S32_RETARD": "1:on",
    "S32_DIP": "2:3=on", "S32_PUMP": "2=on", "S32_DIGIN": "4=lo", "S32_SWITCH": "3:select=1,4:1:plead=stop",
    "S32_BOUNCE_MS": "5", "S32_BOUNCE_EDGES": "3", "S32_PIN_FAULT": "1:RC13=gnd@2-3",
    "S32_ECU_POWER": "1:off@4,1:on@5", "S32_SUPPLY": "2:9.5@3", "S32_BOR_V": "5.5", "S32_RESET": "ct@2",
    "S32_XTAL_PPM": "1=20", "S32_ECU_LINK": "1:ber=0.01@1-2", "S32_LINK_TABLE_SEAL": "full",
    "S32_CONSOLE_IN": "1:1:1:get CONFIG2", "S32_DEV_FAULT": "ct:sd=readerr@1", "S32_RTC": "osf",
    "S32_RTC_EPOCH": "1500000000", "S32_CT_SD_WP": "1", "S32_ECU_SD_WP": "1", "S32_INJ_DEAD": "1.1",
    "S32_INPUT": "1:menu,2:enc:1",
}


@unittest.skipUnless(PAIR.is_file(), "s32pair_shim not built")
class TestEveryFieldChangesSignature(unittest.TestCase):
    """The runtime half of the completeness guard: each knob-reachable field, one pair of runs each."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.tmp = tempfile.TemporaryDirectory()
        root = pathlib.Path(cls.tmp.name)
        cls.fw = conformance.synthetic_firmware(root / "fw")
        cls.sd = root / "ecu1_sd.img"                       # a small card: every block reads back as erased
        cls.sd.write_bytes(b"\xff" * (1024 * 1024))
        cls.bases = {}

    @classmethod
    def tearDownClass(cls) -> None:
        cls.tmp.cleanup()

    def stim(self, knobs: dict) -> dict:
        return stimulus(knobs, self.fw, pathlib.Path(self.tmp.name))

    def check_differs(self, ka: dict, kb: dict, exactly_one: bool, static_same: bool = True) -> None:
        a, b = self.stim(ka), self.stim(kb)
        if static_same:
            self.assertEqual(a["static"], b["static"])    # these fields are stimulus, not static configuration
        self.assertEqual(len(a["items"]), len(b["items"]))
        differing = [i for i, (x, y) in enumerate(zip(a["items"], b["items"])) if x != y]
        if exactly_one:
            self.assertEqual(len(differing), 1, f"items of {ka!r} vs {kb!r}")
        else:
            self.assertTrue(differing, f"items of {ka!r} vs {kb!r}")

    def test_every_knob_reachable_field_changes_the_items(self) -> None:
        for struct, pairs in KNOB_PAIRS.items():
            for field, pair in pairs.items():
                with self.subTest(struct=struct, field=field):
                    ka, kb = pair[0], pair[1]
                    static_same = pair[2] if len(pair) > 2 else True
                    self.check_differs(ka, kb, exactly_one=struct != "sb_ev" or field != "bit",
                                       static_same=static_same)

    def test_engine_and_stats_items(self) -> None:
        for what, (ka, kb) in EXTRA_PAIRS.items():
            with self.subTest(item=what):
                self.check_differs(ka, kb, exactly_one=True)

    def test_every_knob_moves_the_signature(self) -> None:
        for knob, sample in KNOB_SAMPLES.items():
            with self.subTest(knob=knob):
                extra = {"S32_ECU1_SD": str(self.sd)} if knob == "S32_ECU_SD_WP" else {}
                key = tuple(sorted(extra.items()))
                if key not in self.bases:
                    self.bases[key] = self.stim(extra)
                base = self.bases[key]
                s = self.stim({**extra, knob: sample})
                self.assertNotEqual((base["static"], base["items"]), (s["static"], s["items"]),
                                    f"{knob} changed nothing the signature sees")


# ------------------------------------------------------------------ source-derived checks
HDR = REPO / "emu" / "common" / "s32board.h"
SRC = REPO / "emu" / "common" / "s32board.c"
PAIR_SRC = REPO / "emu" / "pair" / "s32pair.c"
TEST_BOARD = REPO / "emu" / "pair" / "test" / "test_board.c"


def struct_fields(name: str) -> list:
    """The field names of a one-typedef struct in s32board.h, in declaration order."""
    m = re.search(r"typedef struct \{(?P<body>[^{}]*?)\}\s*" + name + r"\s*;", HDR.read_text(), re.S)
    if not m:
        raise AssertionError(f"no struct {name} in {HDR}")
    body = re.sub(r"/\*.*?\*/", "", m.group("body"), flags=re.S)
    fields = []
    for decl in body.split(";"):
        decl = decl.strip()
        if not decl:
            continue
        _type, _, names = decl.partition(" ")
        fields += [re.sub(r"\[.*\]", "", x.strip()) for x in names.split(",")]
    return fields


def function_body(src: str, name: str) -> str:
    m = re.search(r"(?:static )?void " + name + r"\(.*?\n\}", src, re.S)
    if not m:
        raise AssertionError(f"no function {name} found")
    return m.group(0)


class TestFieldListsCannotGoStale(unittest.TestCase):
    """The anti-staleness half: the field lists above and in test_board.c are checked against the structs themselves
    and against what the item serializers reference, so a field added without hashing fails here, naming it."""

    def test_every_struct_field_is_accounted_for(self) -> None:
        for struct in STRUCTS:
            fields = struct_fields(struct)
            accounted = set(KNOB_PAIRS[struct]) | C_ONLY[struct] | NEVER_HASHED.get(struct, set())
            self.assertEqual(set(fields), accounted,
                             f"{struct}: not in any list: {set(fields) - accounted}; stale entries: "
                             f"{accounted - set(fields)}")

    def test_every_field_is_referenced_by_its_item_serializer(self) -> None:
        src = SRC.read_text()
        missing = []
        for struct in STRUCTS:
            body = function_body(src, ITEM_FN[struct])
            referenced = set(re.findall(r"[a-z]->(\w+)", body))
            for field in struct_fields(struct):
                if field in NEVER_HASHED.get(struct, set()):
                    if field in referenced:
                        missing.append(f"{struct}.{field} must stay out of the hash (see NEVER_HASHED)")
                elif field not in referenced:
                    missing.append(f"{struct}.{field} is not hashed by {ITEM_FN[struct]}")
        self.assertEqual(missing, [])

    def test_c_unit_test_tables_match_the_structs(self) -> None:
        test_src = TEST_BOARD.read_text()
        for struct in STRUCTS:
            table = set(re.findall(r"SF\(" + struct + r",\s*(\w+),\s*[01]\)", test_src))
            self.assertEqual(set(struct_fields(struct)), table,
                             f"emu/pair/test/test_board.c {struct} table: update it and make sure "
                             f"{ITEM_FN[struct]} hashes the new field")

    def test_engine_and_stats_items_serialized(self) -> None:
        body = function_body(SRC.read_text(), "s32board_stimulus")
        self.assertIn("b->rpm", body)
        self.assertIn("b->start_s", body)
        pair = PAIR_SRC.read_text()
        self.assertRegex(pair, r"s32snap_item\(&SIG_ITEMS, 6, 0, STATS_FROM, &STATS_FROM")


if __name__ == "__main__":
    unittest.main()
