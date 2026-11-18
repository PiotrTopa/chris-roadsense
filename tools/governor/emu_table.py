#!/usr/bin/env python3
"""emu_table.py DIR REF RUN [RUN ...] [--from S] [--to S]: a Markdown table of emu.py runs (names without
extension, in DIR): the emulated platform (NAME.plant.jsonl) and what the consumer got against the reference run
(tools/governor/compare.py on NAME.jsonl)."""
import json
import os
import statistics as st
import subprocess
import sys


def arg(flag, default=None):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


T0, T1 = float(arg("--from", "150")), float(arg("--to", "1e9"))
names = []
skip = False
for a in sys.argv[1:]:
    if skip:
        skip = False
    elif a in ("--from", "--to"):
        skip = True
    else:
        names.append(a)
D, names = names[0], names[1:]
here = os.path.dirname(os.path.abspath(__file__))
cmp_ = json.loads(subprocess.run([sys.executable, os.path.join(here, "compare.py")] + [os.path.join(D, n + ".jsonl") for n in names]
                                 + ["--from", str(T0), "--to", str(T1), "--json"], capture_output=True, text=True).stdout)
rows = []


def row(label, f):
    rows.append("| " + label + " | " + " | ".join(str(f(n)) for n in names) + " |")


plant, res = {}, {}
for n in names:
    p = [json.loads(l) for l in open(os.path.join(D, n + ".plant.jsonl"))]
    plant[n] = [x for x in p if T0 <= x["t"] <= T1]
    plant[n + "/all"] = p
    res[n] = cmp_[os.path.join(D, n + ".jsonl")]


def pm(n, key, f=st.mean, nd=1):
    v = [x[key] for x in plant[n] if x.get(key) is not None]
    return round(f(v), nd) if v else "-"


def levels(n):
    c = {}
    for x in plant[n]:
        c[x["level"]] = c.get(x["level"], 0) + 1
    return ", ".join("%s: %d %%" % (k, round(100 * v / len(plant[n]))) for k, v in sorted(c.items(), key=lambda kv: str(kv[0])))


def dl(n, topic, key):
    return (res[n]["delivered"].get(topic) or {}).get(key, "-")


def err(n, name, key="med"):
    if n == names[0]:
        return "ref"
    e = res[n]["error_vs_ref"].get(name)
    return "-" if not e else "%s / %s" % (e.get("med"), e.get("p95")) if key == "med" else e.get(key)


print("| | " + " | ".join(names) + " |")
print("|---|" + "---|" * len(names))
row("emulated temperature, mean / max over the whole run, C", lambda n: "%s / %s" % (pm(n, "temp"), max(x["temp"] for x in plant[n + "/all"])))
row("emulated power, W", lambda n: pm(n, "p_w", nd=2))
row("limiter: share of the time cutting / mean cut of the CPU quota", lambda n: "%d %% / %s %%" % (round(100 * sum(1 for x in plant[n] if x["cut_pct"] > 0) / len(plant[n])), pm(n, "cut_pct")))
row("governor level, share of the time", levels)
row("road: messages a second", lambda n: dl(n, "road", "hz"))
row("road: latency camera -> consumer, median / p90 / p99, ms", lambda n: "%s / %s / %s" % (dl(n, "road", "lat_med"), dl(n, "road", "lat_p90"), dl(n, "road", "lat_p99")))
row("road: longest gap between two messages, ms", lambda n: dl(n, "road", "gap_max_ms"))
row("road: model ms", lambda n: dl(n, "road", "model_ms"))
row("`lead` / `lanes` marked degraded, % of messages", lambda n: "%s / %s" % (dl(n, "lead", "degraded_pct"), dl(n, "lanes", "degraded_pct")))
row("signs: messages a second (marked degraded, %)", lambda n: "%s (%s)" % (dl(n, "signs", "hz"), dl(n, "signs", "degraded_pct")))
row("lights: messages a second (marked degraded, %)", lambda n: "%s (%s)" % (dl(n, "lights", "hz"), dl(n, "lights", "degraded_pct")))
row("env / view: messages a second", lambda n: "%s / %s" % (dl(n, "env", "hz"), dl(n, "view", "hz")))
row("ego lines of `road` against the reference at 9 m, median / p95 of dy, m", lambda n: err(n, "road R y@9"))
row("... at 32 m", lambda n: err(n, "road R y@32"))
row("... at 61 m", lambda n: err(n, "road R y@61"))
row("tracked right line of `lanes` at 32 m, median / p95, m", lambda n: err(n, "lanes R y@32"))
row("tracked lead distance, median / p95, relative", lambda n: err(n, "lead dist"))
row("tracked lead relative speed, median / p95, m/s", lambda n: err(n, "lead vrel"))
row("lead presence disagreeing with the reference, % of frames", lambda n: err(n, "lead present", "disagree_pct"))
print("\n".join(rows))
