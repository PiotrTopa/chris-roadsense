#!/usr/bin/env python3
"""compare.py REF.jsonl RUN.jsonl [RUN2.jsonl ...] [--from S] [--to S] [--json]

Traces of tools/governor/trace.py of the same replayed ride (camerad-replay from its first frame, so a frame
number names the same picture in every run): what each run delivered, and how far its road / lead / lanes
outputs are from the reference run's on the same frames. The reference values are interpolated between the
reference run's messages by frame number (the model runs on whichever frames it reaches, so two runs rarely
share frames); the same comparison of a second reference run gives the run-to-run noise.

Per run, over the window [--from, --to] seconds of the trace:
  road: messages a second, share of camera frames with a road run, latency camera -> consumer median / p90 /
        p99 / max, longest gap between two road messages, model ms;
  error against the reference: ego lane lines at 9 / 32 / 61 m (median / p95 of |dy|, where both runs have the
        line at prob > 0.5), lead distance (median / p95 relative, both tracked), lead presence agreement,
        tracked lines of `lanes` at 9 / 32 m;
  platform: hottest zone mean / max, rail W, clock cuts (share of the time a cooling device was active, mean
        big-core and GPU clock), governor level (time at each level), marker counts.
"""
import bisect
import json
import statistics as st
import sys


def arg(flag, default=None):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


T0, T1 = float(arg("--from", "0")), float(arg("--to", "1e9"))
FILES = [a for i, a in enumerate(sys.argv[1:], 1) if not a.startswith("--") and sys.argv[i - 1] not in ("--from", "--to")]


def load(path):
    by = {}
    with open(path) as f:
        for ln in f:
            try:
                d = json.loads(ln)
            except ValueError:
                continue
            by.setdefault(d.get("k"), []).append(d)
    return by


def pct(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p * (len(v) - 1) + 0.5))] if v else None


def r(x, n=2):
    return None if x is None else round(x, n)


class Interp:
    """Reference series by frame number; value(seq) linear between the neighbours when they are at most
    max_gap frames apart, else None."""

    def __init__(self, pts, max_gap=6):
        pts = sorted((s, v) for s, v in pts if s is not None and v is not None)
        self.s = [p[0] for p in pts]
        self.v = [p[1] for p in pts]
        self.max_gap = max_gap

    def __call__(self, seq):
        i = bisect.bisect_left(self.s, seq)
        if i < len(self.s) and self.s[i] == seq:
            return self.v[i]
        if i == 0 or i >= len(self.s) or self.s[i] - self.s[i - 1] > self.max_gap:
            return None
        a = (seq - self.s[i - 1]) / (self.s[i] - self.s[i - 1])
        return self.v[i - 1] + a * (self.v[i] - self.v[i - 1])


def series(tr):
    """name -> [(seq, value)] of everything compared."""
    out = {}

    def add(name, seq, v):
        out.setdefault(name, []).append((seq, v))

    for d in tr.get("road", []):
        lp = d.get("lp") or [None, None]
        for side, key, p in (("L", "ly", lp[0]), ("R", "ry", lp[1])):
            y = d.get(key)
            ok = y is not None and p is not None and p > 0.5
            for j, x in enumerate((9, 32, 61)):
                add("road %s y@%d" % (side, x), d["seq"], y[j] if ok else None)
        ld = d.get("ld") or [None, None]
        add("road lead prob", d["seq"], ld[0])
        add("road lead x", d["seq"], ld[1] if ld[0] is not None and ld[0] > 0.5 else None)
    for d in tr.get("lead", []):
        add("lead present", d["seq"], 1.0 if d.get("present") else 0.0)
        add("lead dist", d["seq"], d.get("dist") if d.get("present") else None)
        add("lead vrel", d["seq"], d.get("vrel") if d.get("present") else None)
    for d in tr.get("lanes", []):
        for side in ("l", "r"):
            v = d.get(side) or [None, None, None]
            add("lanes %s present" % side.upper(), d["seq"], 1.0 if v[0] else 0.0)
            add("lanes %s y@9" % side.upper(), d["seq"], v[1] if v[0] else None)
            add("lanes %s y@32" % side.upper(), d["seq"], v[2] if v[0] else None)
    return out


def window(tr, k):
    return [d for d in tr.get(k, []) if T0 <= d.get("t", 0) <= T1]


def delivered(tr):
    out = {}
    road = window(tr, "road")
    cam = window(tr, "camera")
    if road:
        span = road[-1]["t"] - road[0]["t"] or 1.0
        lat = [d["lat"] for d in road if d.get("lat") is not None]
        gaps = [b["t"] - a["t"] for a, b in zip(road, road[1:])]
        frames = (road[-1]["seq"] - road[0]["seq"]) or 1
        out["road"] = {"hz": r((len(road) - 1) / span), "frames_covered_pct": r(100.0 * (len(road) - 1) / frames, 1),
                       "lat_med": r(st.median(lat), 1), "lat_p90": r(pct(lat, 0.9), 1), "lat_p99": r(pct(lat, 0.99), 1),
                       "lat_max": r(max(lat), 1), "gap_max_ms": r(1000 * max(gaps), 0) if gaps else None,
                       "gap_p99_ms": r(1000 * pct(gaps, 0.99), 0) if gaps else None,
                       "model_ms": r(st.mean(d["model_ms"] for d in road if d.get("model_ms")), 1),
                       "capped_pct": r(100.0 * sum(1 for d in road if (d.get("cap") or 99) < 19.9) / len(road), 1)}
    for k in ("lead", "lanes", "lights", "signs", "env", "view", "camera"):
        m = window(tr, k)
        if len(m) > 1:
            span = m[-1]["t"] - m[0]["t"] or 1.0
            lat = [d["lat"] for d in m if d.get("lat") is not None]
            deg = [d for d in m if (d.get("rate") or {}).get("degraded")]
            why = {}
            for d in m:
                for w in (d.get("rate") or {}).get("why") or []:
                    why[w] = why.get(w, 0) + 1
            out[k] = {"hz": r((len(m) - 1) / span), "lat_med": r(st.median(lat), 1) if lat else None,
                      "lat_p90": r(pct(lat, 0.9), 1) if lat else None,
                      "degraded_pct": r(100.0 * len(deg) / len(m), 1), "why": why}
    sysl = window(tr, "sys")
    if sysl:
        hot = [max(v for v in d["zones"].values() if v is not None and v < 150) for d in sysl]
        big = [st.mean(v for n, v in d["zones"].items() if n in ("cpu4", "cpu5", "cpu6", "cpu7") and v is not None)
               for d in sysl if any(n == "cpu4" for n in d["zones"])]
        w = [d["w"] for d in sysl if d.get("w") is not None]
        cool = [any((v or 0) > 0 for v in d["cool"].values()) for d in sysl]
        coolmax = {}
        for d in sysl:
            for n, v in d["cool"].items():
                coolmax[n] = max(coolmax.get(n, 0), v or 0)
        lv = {}
        for d in sysl:
            L = (d.get("gov") or {}).get("level")
            lv[L] = lv.get(L, 0) + 1
        fr = {}
        for d in sysl:
            for n, v in (d.get("freq") or {}).items():
                if v:
                    fr.setdefault(n, []).append(v)
        out["platform"] = {"hottest_mean": r(st.mean(hot), 1), "hottest_max": r(max(hot), 1),
                           "big_mean": r(st.mean(big), 1) if big else None,
                           "rail_w": r(st.mean(w), 2) if w else None,
                           "limiter_active_pct": r(100.0 * sum(cool) / len(cool), 1),
                           "limiter_max_state": {n: v for n, v in coolmax.items() if v},
                           "clock_mean": {n: r(st.mean(v) / (1e6 if n == "gpu" else 1e3), 0) for n, v in fr.items()},
                           "level_time_pct": {str(k): r(100.0 * v / len(sysl), 1) for k, v in sorted(lv.items(), key=lambda kv: str(kv[0]))},
                           "headroom_min": min([(d.get("gov") or {}).get("headroom_k") for d in sysl
                                                if (d.get("gov") or {}).get("headroom_k") is not None] or [None])}
    out["aborted"] = bool(tr.get("abort"))
    return out


def errors(ref_series, tr):
    ser = series(tr)
    tsel = {}
    for k in ("road", "lead", "lanes"):
        for d in window(tr, k):
            tsel.setdefault(k, set()).add(d["seq"])
    out = {}
    for name, pts in ser.items():
        if name not in ref_series:
            continue
        top = name.split()[0]
        ref = Interp(ref_series[name])
        # presence series: the reference's presence at the frame is its nearest message's
        diffs, both, only_run, only_ref, n = [], 0, 0, 0, 0
        for seq, v in pts:
            if seq not in tsel.get(top, ()):
                continue
            rv = ref(seq)
            n += 1
            if name.endswith("present"):
                if rv is None:
                    continue
                diffs.append(abs((1.0 if rv >= 0.5 else 0.0) - v))
                continue
            if v is not None and rv is not None:
                both += 1
                diffs.append(abs(v - rv) / max(1.0, abs(rv)) if name in ("lead dist", "road lead x") else abs(v - rv))
            elif v is not None:
                only_run += 1
            elif rv is not None:
                only_ref += 1
        if not diffs:
            continue
        if name.endswith("present"):
            out[name] = {"disagree_pct": r(100.0 * sum(diffs) / len(diffs), 2), "n": len(diffs)}
        else:
            out[name] = {"med": r(st.median(diffs), 3), "p95": r(pct(diffs, 0.95), 3), "n": both,
                         "only_run": only_run, "only_ref": only_ref}
    return out


def main():
    ref = load(FILES[0])
    ref_series = series(ref)
    res = {}
    for i, p in enumerate(FILES):
        tr = ref if i == 0 else load(p)
        res[p] = {"delivered": delivered(tr), "error_vs_ref": {} if i == 0 else errors(ref_series, tr)}
    if "--json" in sys.argv:
        print(json.dumps(res, indent=1))
        return
    for p, d in res.items():
        print("==", p, "(reference)" if p == FILES[0] else "")
        for k, v in d["delivered"].items():
            print("  %-9s %s" % (k, json.dumps(v)))
        for k, v in sorted(d["error_vs_ref"].items()):
            print("  err %-18s %s" % (k, json.dumps(v)))


if __name__ == "__main__":
    main()
