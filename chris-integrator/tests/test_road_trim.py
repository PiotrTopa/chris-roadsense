"""fastjson.trim_road: the `road` message without the members the default stack does not read, and nothing else
changed; the default stack gives the same outputs from the trimmed message as from the whole one."""
import json
import os
import unittest

from chris_integrator import fastjson as F
from chris_integrator.io.sockets import parse_line, parse_line_trimmed
from chris_integrator.mixer import MessageClock
from chris_integrator.stack import build

DATA = os.path.join(os.path.dirname(__file__), "data", "road_raw_vision.jsonl")   # the vision model's layout, synthetic values (data/make_road_raw.py)


def lines():
    with open(DATA, "rb") as f:
        return [ln.strip() for ln in f if ln.strip()]


def expected(raw):
    m = json.loads(raw)
    for k in F.PLAN_UNREAD:
        if isinstance(m["plan"].get(k), list):
            del m["plan"][k]
    for key in ("lane_lines", "road_edges"):
        for ln in m[key]:
            for k in F.LINE_UNREAD:
                ln.pop(k, None)
    return m


class TestRoadTrim(unittest.TestCase):
    def test_only_the_listed_members_go(self):
        for raw in lines():
            cut = F.trim_road(raw)
            self.assertLess(len(cut), 0.65 * len(raw))
            self.assertEqual(json.loads(cut), expected(raw))
            msg, line = parse_line_trimmed(raw)
            self.assertEqual(msg, expected(raw))
            self.assertEqual(line, raw.decode())               # the line handed on is the whole one

    def test_other_layouts_are_left_alone(self):
        whole = {"type": "road", "seq": 1, "lane_lines": [], "road_edges": [{"x": [0, 1], "y": [1, 2], "z": [3, 4]}],
                 "plan": {"x": [0, 1], "y": [0, 0], "z": [5, 6], "vx": [1, 1], "prob": None},
                 "leads": [{"x": [1.0], "z": 'text ,"z":[ in a string'}], "note": ',"plan":{"vx":[1]}'}
        raw = json.dumps(whole, separators=(",", ":")).encode()
        got = F.loads_road_trimmed(raw)
        self.assertEqual(got["plan"], {"x": [0, 1], "y": [0, 0], "z": [5, 6], "prob": None})      # plan.z stays
        self.assertEqual(got["road_edges"], [{"x": [0, 1], "y": [1, 2]}])
        self.assertEqual(got["leads"], whole["leads"])
        self.assertEqual(got["note"], whole["note"])
        for odd in (b'{"type":"road","plan":{"vx":[1,2', b'{"type":"road","lane_lines":[{"z":[1]}]}', b'{"type":"road"}'):
            try:
                want = json.loads(odd)
            except ValueError:
                want = None
            if want is None:
                self.assertRaises(ValueError, F.loads_road_trimmed, odd)
            else:
                self.assertEqual(F.loads_road_trimmed(odd), want)
        self.assertEqual(parse_line_trimmed(b'{"type":"camera","seq":3}'), parse_line(b'{"type":"camera","seq":3}'))

    def test_default_stack_reads_none_of_them(self):
        outs = []
        for parse in (parse_line, parse_line_trimmed):
            st = build(clock=MessageClock(), mapper=None)
            self.assertTrue(st.trim_road)
            got = []
            st.mixer.add_output("t", lambda e: got.append((e.stream, json.dumps(e.msg, sort_keys=True))) if e.stream != "road" else None)
            for k, raw in enumerate(lines() * 4):
                m = json.loads(raw)
                m["seq"] += 100 * k
                m["ts_mono_ns"] += k * 66_000_000
                msg, line = parse(json.dumps(m, separators=(",", ":")).encode())
                st.deliver(msg, line)
            outs.append(got)
        self.assertGreater(len(outs[0]), 30)
        self.assertEqual(outs[0], outs[1])

    def test_a_stack_with_its_own_boxes_gets_the_whole_message(self):
        self.assertFalse(build(clock=MessageClock(), mapper=None, cfg={"mixer": {"road_whole": True}}).trim_road)
        self.assertFalse(build(clock=MessageClock(), mapper=None,
                               cfg={"mixer": {"replace": {"fused": {"class": "chris_integrator.plugins.trackers:Fused"}}}}).trim_road)


if __name__ == "__main__":
    unittest.main()
