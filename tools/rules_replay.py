#!/usr/bin/env python3
"""Run the api's rules-in-effect engine (and the speed limit inside it) offline over a recorded /ws stream.

    python3 tools/rules_replay.py rec.jsonl [--ride SLUG] [--no-lights] [--every 10] [--out timeline.json]

Input: JSONL, one message per line, either raw api messages or the recorder's {"t": <laptop time>, "m": <message>}
wrapper (lane scratch scripts). `camera` messages split the stream into rides (camerad-replay --playlist: `ride`,
`ride_t_s`); the engine is reset on a ride change like the api does. Times printed are ride seconds (from the
camera message's ride_t_s, interpolated with the monotonic clock) or seconds from the first message.
--out writes, per ride, the intervals of every rule (start/end ride time, odometer, end reason), the passed
momentary signs, the warnings, the junction cues and the ignored tracks, for scoring against human labels.
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "chris-integrator"))
from chris_integrator.trackers.rules import RulesInEffect  # noqa: E402
from chris_integrator.trackers.speedlimit import load_rules  # noqa: E402


class RideRun:
    def __init__(self, slug, rules, cfg, quiet):
        self.slug, self.quiet = slug, quiet
        self.eng = RulesInEffect(rules, cfg=dict(cfg))
        self.eng.ignored = []              # unbounded offline (the api keeps the last 30)
        self.eng.speed.ignored = []
        self.cam = None                    # (ts_mono_ns, ride_t_s)
        self.t0 = None
        self.active = {}                   # (id, track) -> start record
        self.intervals, self.passed, self.warnings, self.junctions, self.ignored = [], [], [], [], []
        self.speed_intervals, self._speed_cur = [], None
        self.n_msgs, self.cost_ns = 0, 0
        self._seen_recent, self._seen_ahead, self._seen_junc, self._n_ign, self._n_sign = set(), set(), set(), 0, 0

    def t_ride(self, ts):
        if self.cam:
            return self.cam[1] + (ts - self.cam[0]) / 1e9
        return (ts - self.t0) / 1e9 if self.t0 is not None else 0.0

    def feed(self, msg):
        ts = msg.get("ts_mono_ns")
        if ts is None:
            return
        if self.t0 is None:
            self.t0 = ts
        if msg.get("type") == "camera":
            if msg.get("ride_t_s") is not None:
                self.cam = (ts, float(msg["ride_t_s"]))
            return
        e = self.eng
        c0 = time.perf_counter_ns()
        e.on_message(msg)
        self.cost_ns += time.perf_counter_ns() - c0
        self.n_msgs += 1
        t = self.t_ride(ts)
        # lasting / until-junction rules: intervals keyed by (id, first track)
        now = {(rid, r["info"]["track_id"]): r for rid, r in e.active.items()}
        for key, r in now.items():
            if key not in self.active:
                self.active[key] = {"id": key[0], "sign": r["sign"], "name": r["name"], "track_id": key[1],
                                    "start_t": round(t, 1), "start_odo": round(e.odo, 1), "side": r["side"],
                                    "uncertain": r["uncertain"], "confirmations": 1}
                self.say(t, e.odo, f"START {r['sign']} {r['name']} (track {key[1]}" + (f", {r['side']}" if r["side"] else "") + ")")
            else:
                self.active[key]["confirmations"] = r["confirmations"]
        for key in [k for k in self.active if k not in now]:
            rec = self.active.pop(key)
            why = next((c["event"].split("ended: ", 1)[1] for c in reversed(e.changes)
                        if c["event"].startswith(f"{rec['sign']} {rec['name']} ended: ")), "?")
            rec.update(end_t=round(t, 1), end_odo=round(e.odo, 1), end_reason=why)
            self.intervals.append(rec)
            self.say(t, e.odo, f"END   {rec['sign']} {rec['name']}: {why}")
        # speed limit as one more rule
        val, src, conf, _ = e.speed.current()
        skey = (val, src.get("kind"), src.get("track_id"))
        if skey != (self._speed_cur or {}).get("key"):
            if self._speed_cur:
                self._speed_cur.update(end_t=round(t, 1), end_odo=round(e.odo, 1))
                self.speed_intervals.append({k: v for k, v in self._speed_cur.items() if k != "key"})
            self._speed_cur = {"key": skey, "limit_kmh": val, "kind": src.get("kind"), "track_id": src.get("track_id"),
                               "rule": src.get("rule"), "start_t": round(t, 1), "start_odo": round(e.odo, 1)}
            self.say(t, e.odo, f"SPEED {val} ({src.get('kind')} {src.get('sign') or src.get('rule') or ''})")
        for r in e.recent:
            tid = r["info"]["track_id"]
            if tid not in self._seen_recent and not r.get("until_junction"):
                self._seen_recent.add(tid)
                self.passed.append({"id": r["id"], "sign": r["sign"], "name": r["name"], "track_id": tid, "t": round(t, 1),
                                    "odo": round(e.odo, 1), "lateral_m": r["info"].get("lateral_m"), "uncertain": r["uncertain"],
                                    "applies_to_car": r["applies_to_car"], "score": r["info"]["score"], "hits": r["info"]["hits"]})
                self.say(t, e.odo, f"PASS  {r['sign']} {r['name']} (track {tid}, lat {r['info'].get('lateral_m')})")
        for w in e.ahead:
            tid = w["info"]["track_id"]
            if tid not in self._seen_ahead:
                self._seen_ahead.add(tid)
                self.warnings.append({"id": w["id"], "sign": w["sign"], "name": w["name"], "track_id": tid, "t": round(t, 1),
                                      "odo": round(e.odo, 1)})
                self.say(t, e.odo, f"AHEAD {w['sign']} {w['name']} (track {tid})")
        for j in e.junctions:
            if id(j) not in self._seen_junc:
                self._seen_junc.add(id(j))
                self.junctions.append({"t": round(t, 1), "odo": j["odo"], "source": j["source"], "detail": j["detail"]})
                self.say(t, e.odo, f"JUNC  {j['source']} at odo {j['odo']:.0f} ({j['detail']})")
        while self._n_ign < len(e.ignored):
            i = e.ignored[self._n_ign]
            self._n_ign += 1
            self.ignored.append(dict(i, t=round(t, 1), engine="rules"))
            self.say(t, e.odo, f"IGN   #{i['track_id']} {i['cls']}: {i['reason']}")
        while self._n_sign < len(e.speed.ignored):
            i = e.speed.ignored[self._n_sign]
            self._n_sign += 1
            self.ignored.append(dict(i, t=round(t, 1), engine="speed"))
            self.say(t, e.odo, f"IGN-S #{i['track_id']} {i['cls']}: {i['reason']}")

    def say(self, t, odo, text):
        if not self.quiet:
            print(f"{self.slug:22s} {t:7.1f}s  odo {odo:6.0f}  {text}")

    def finish(self):
        e = self.eng
        t = self.t_ride(e.now_ns) if e.now_ns else 0.0
        for key, rec in list(self.active.items()):
            rec.update(end_t=round(t, 1), end_odo=round(e.odo, 1), end_reason="ride end")
            self.intervals.append(rec)
        if self._speed_cur:
            self._speed_cur.update(end_t=round(t, 1), end_odo=round(e.odo, 1))
            self.speed_intervals.append({k: v for k, v in self._speed_cur.items() if k != "key"})
        return {"ride": self.slug, "duration_s": round(t, 1), "odometer_m": round(e.odo, 1), "messages": self.n_msgs,
                "us_per_message": round(self.cost_ns / max(1, self.n_msgs) / 1000, 1),
                "intervals": self.intervals, "speed_limit": self.speed_intervals, "passed": self.passed,
                "warnings": self.warnings, "junctions": self.junctions, "ignored": self.ignored}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl")
    ap.add_argument("--rules", default="pl")
    ap.add_argument("--ride", help="only this ride slug")
    ap.add_argument("--no-lights", action="store_true", help="traffic lights are not a junction cue")
    ap.add_argument("--no-priority-inference", action="store_true", help="pre-registered engine: no turn ending / A-6 refresh of D-1")
    ap.add_argument("--cx", type=float, default=640.0)
    ap.add_argument("--focal", type=float, help="override the focal (default: per-ride camera-replay.json next to --rides-dir)")
    ap.add_argument("--rides-dir", default=os.path.expanduser("~/datasets/rides"))
    ap.add_argument("--out", help="write per-ride timelines as JSON")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()
    rules = load_rules(a.rules)
    base_cfg = {"cx": a.cx, "width": 1280, "lights_junction": not a.no_lights, "priority_inference": not a.no_priority_inference}
    runs, cur, cur_key = [], None, None
    with open(a.jsonl) as f:
        for line in f:
            try:
                m = json.loads(line)
            except ValueError:
                continue
            msg = m.get("m") if "m" in m and "t" in m else m
            if not isinstance(msg, dict):
                continue
            if msg.get("type") == "camera":
                key = (msg.get("ride"), msg.get("ride_index"))
                seq = msg.get("seq") or 0
                if key != cur_key or (cur is not None and seq < getattr(cur, "last_seq", 0)):
                    if cur is not None:
                        runs.append(cur.finish())
                    slug = msg.get("ride") or "stream"
                    cfg = dict(base_cfg)
                    focal = a.focal
                    if focal is None:
                        try:
                            with open(os.path.join(a.rides_dir, slug, "camera-replay.json")) as cf:
                                cj = json.load(cf)
                            focal, cfg["cx"] = float(cj.get("focal_px", 690.0)), float(cj.get("cx", a.cx))
                        except (OSError, ValueError):
                            focal = 690.0
                    cfg["focal_px"] = focal
                    cur, cur_key = RideRun(slug, rules, cfg, a.quiet or (a.ride and slug != a.ride)), key
                    cur.skip = bool(a.ride and slug != a.ride)
                cur.last_seq = seq
            if cur is None:
                cur, cur_key = RideRun("stream", rules, dict(base_cfg, focal_px=a.focal or 690.0), a.quiet), None
                cur.skip = False
            if not cur.skip:
                cur.feed(msg)
    if cur is not None:
        runs.append(cur.finish())
    runs = [r for r in runs if not a.ride or r["ride"] == a.ride]
    for r in runs:
        print(f"== {r['ride']}: {r['duration_s']:.0f} s, {r['odometer_m']:.0f} m, {r['messages']} msgs, "
              f"{r['us_per_message']} us/msg, {len(r['intervals'])} rule intervals, {len(r['passed'])} passed, "
              f"{len(r['warnings'])} warnings, {len(r['junctions'])} junction cues, {len(r['ignored'])} ignored")
    if a.out:
        with open(a.out, "w") as f:
            json.dump(runs, f, indent=1)


if __name__ == "__main__":
    main()
