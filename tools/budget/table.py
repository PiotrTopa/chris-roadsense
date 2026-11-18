#!/usr/bin/env python3
"""table.py OUTDIR NAME[=label] ... : Markdown tables from run.sh results (budget, /ws probe, daemon stats
lines): one column per run. Rows: phone watts, temperatures, CPU on the little and big cores per daemon and
stage (thread names), GPU time per daemon, message rate and latency per topic, the daemons' own timings."""
import json, os, re, sys
out = sys.argv[1]; runs = [a.split("=", 1) if "=" in a else [a, a] for a in sys.argv[2:]]

def load(n):
    d = json.loads(open(os.path.join(out, n + ".budget")).read().split("BUDGET ", 1)[1])
    try: ws = json.loads(open(os.path.join(out, n + ".ws")).read().split("WS ", 1)[1])
    except (OSError, IndexError, ValueError): ws = {}
    logs = {}
    for u in ("modeld", "signd", "envd", "camerad"):
        try: logs[u] = open(os.path.join(out, "%s.%s.log" % (n, u))).read()
        except OSError: logs[u] = ""
    return d, ws, logs

def mean(pat, text):
    v = [float(x) for x in re.findall(pat, text)]
    return round(sum(v) / len(v), 2) if v else None

R = [load(n) for n, _ in runs]
def row(name, vals): print("| %s | %s |" % (name, " | ".join("-" if v is None else str(v) for v in vals)))
print("| | %s |" % " | ".join(l for _, l in runs)); print("|---|%s" % ("---|" * len(runs)))
row("phone rail, W (mean / last 60 s)", ["%.2f / %.2f" % (d["watts_mean"], d["watts_last60"]) for d, _, _ in R])
row("temperature last 60 s, C: big cores / little / GPU", ["%.0f / %.0f / %.0f" % (d["temp_last60"]["big"], d["temp_last60"]["little"], d["temp_last60"]["gpu"]) for d, _, _ in R])
row("hottest zone, max over the run, C", [d["temp_max"] for d, _, _ in R])
row("CPU, % of one core: little cores / big cores", ["%.0f / %.0f" % (d["cpu_little_pct"], d["cpu_big_pct"]) for d, _, _ in R])
row("GPU busy, % of wall (sum of the daemons' GPU time)", [d["gpu_pct"] for d, _, _ in R])
units = []
for d, _, _ in R:
    for u in d["units"]:
        if u not in units: units.append(u)
for u in units:
    row("%s: little / big / GPU" % u.replace("l21-", ""), [("%.1f / %.1f / %.1f" % (d["units"][u]["cpu_little_pct"], d["units"][u]["cpu_big_pct"], d["units"][u]["gpu_pct"])) if u in d["units"] else None for d, _, _ in R])
    names = []
    for d, _, _ in R:
        for t in d["units"].get(u, {}).get("threads", {}):
            if t not in names: names.append(t)
    if len(names) > 1:
        for t in names:
            row("&nbsp;&nbsp;thread `%s`: little / big" % t, [("%.1f / %.1f" % tuple(d["units"][u]["threads"][t])) if t in d["units"].get(u, {}).get("threads", {}) else None for d, _, _ in R])
print()
print("| topic: msg/s, latency median / p90 ms | %s |" % " | ".join(l for _, l in runs)); print("|---|%s" % ("---|" * len(runs)))
topics = []
for _, ws, _ in R:
    for t in ws:
        if t not in topics: topics.append(t)
for t in topics:
    row(t, [("%.1f/s, %s / %s" % (ws[t]["rate"], ws[t].get("lat_med", "-"), ws[t].get("lat_p90", "-"))) if t in ws else None for _, ws, _ in R])
print()
print("| daemon stats (mean of the 10 s lines) | %s |" % " | ".join(l for _, l in runs)); print("|---|%s" % ("---|" * len(runs)))
row("modeld runs/s", [mean(r"modeld: ([\d.]+) runs/s", l["modeld"]) for _, _, l in R])
row("modeld model ms mean / p90", ["%s / %s" % (mean(r"model ([\d.]+) ms avg", l["modeld"]), mean(r"p90 ([\d.]+)", l["modeld"])) for _, _, l in R])
row("modeld prep ms (per camera frame)", [mean(r"prep ([\d.]+) ms", l["modeld"]) for _, _, l in R])
row("signd signs/s, detect ms (detector)", ["%s, %s (%s)" % (mean(r"signd: ([\d.]+) fps", l["signd"]), mean(r"detect_ms ([\d.]+)", l["signd"]), mean(r"detect_ms [\d.]+ \(det ([\d.]+)\)", l["signd"])) for _, _, l in R])
row("signd lights/s, lights ms (detector)", ["%s, %s (%s)" % (mean(r"lights ([\d.]+) fps", l["signd"]), mean(r"lights_ms ([\d.]+)", l["signd"]), mean(r"lights_ms [\d.]+ \(det ([\d.]+)\)", l["signd"])) for _, _, l in R])
row("envd frames/s, ms per frame", ["%s, %s" % (mean(r"envd: ([\d.]+) fps", l["envd"]), mean(r"env_ms ([\d.]+)", l["envd"])) for _, _, l in R])
