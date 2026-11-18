#!/usr/bin/env python3
"""measure.py NAME SECONDS [--units u1,u2,...] [--extra-pids name=pid,...] [--out DIR]   (root, on the phone)

Budget of a running stack over SECONDS: CPU per unit and per thread (schedstat run time, split into
little cores 0-3 and big cores 4-7 by the core each thread was last seen on at 10 Hz), GPU time per
unit (DRM fdinfo drm-engine-gpu), GPU clock residency (devfreq trans_stat), rail power
(qcom-battery voltage_now x |current_now| at 5 Hz), thermal zones (1 Hz; mean of the last 60 s
and the maximum). Stops the units and reports ABORT when the hottest zone is above 95 C for 3 s.
Prints one JSON line `BUDGET {...}` and writes DIR/NAME.json.
"""
import glob, json, os, subprocess, sys, time

def arg(flag, default=None):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default

NAME, SECS = sys.argv[1], float(sys.argv[2])
UNITS = arg("--units", "l21-camerad,l21-modeld,l21-signd,l21-envd,l21-mapd,l21-api").split(",")
OUT = arg("--out", "/home/user/l21/out")
EXTRA = dict(kv.split("=") for kv in arg("--extra-pids", "").split(",") if kv)
ZONES = {}
for z in glob.glob("/sys/class/thermal/thermal_zone*"):
    try: ZONES[open(z + "/type").read().strip().replace("-thermal", "")] = z + "/temp"
    except OSError: pass
BIG = ["cpu4", "cpu5", "cpu6", "cpu7"]; LITTLE = ["cpu0", "cpu1", "cpu2", "cpu3"]; GPU = ["gpu-top", "gpu-bottom"]

def rd(p):
    try:
        with open(p) as f: return f.read()
    except OSError: return ""

def unit_pids(u):
    cg = subprocess.run(["systemctl", "show", "-p", "ControlGroup", "--value", u], capture_output=True, text=True).stdout.strip()
    return [int(x) for x in rd("/sys/fs/cgroup" + cg + "/cgroup.procs").split()] if cg else []

def threads(pids):
    out = {}
    for pid in pids:
        for t in glob.glob("/proc/%d/task/*" % pid):
            ss = rd(t + "/schedstat").split()
            if not ss: continue
            out[int(os.path.basename(t))] = (rd(t + "/comm").strip(), int(ss[0]), pid)
    return out

def last_cpu(tid, pid):
    s = rd("/proc/%d/task/%d/stat" % (pid, tid))
    try: return int(s[s.rindex(")") + 2:].split()[36])
    except (ValueError, IndexError): return -1

def gpu_ns(pids):
    seen = {}
    for pid in pids:
        for f in glob.glob("/proc/%d/fdinfo/*" % pid):
            s = rd(f)
            if "drm-engine-gpu" not in s: continue
            d = dict(l.split(":", 1) for l in s.splitlines() if ":" in l)
            seen[d["drm-client-id"].strip()] = int(d["drm-engine-gpu"].split()[0])
    return sum(seen.values())

def cpustat():
    out = {}
    for l in rd("/proc/stat").splitlines():
        p = l.split()
        if p[0].startswith("cpu") and p[0] != "cpu":
            v = [int(x) for x in p[1:9]]
            out[p[0]] = (sum(v) - v[3] - v[4], sum(v))
    return out

def trans():
    out = {}
    for l in rd("/sys/class/devfreq/5000000.gpu/trans_stat").splitlines():
        p = l.replace("*", " ").replace(":", " ").split()
        if len(p) > 3 and p[0].isdigit() and p[-1].isdigit(): out[int(p[0]) // 1000000] = int(p[-1])
    return out

def temps():
    return {k: int(rd(p) or 0) / 1000.0 for k, p in ZONES.items()}

def watts():
    v = int(rd("/sys/class/power_supply/qcom-battery/voltage_now") or 0); i = int(rd("/sys/class/power_supply/qcom-battery/current_now") or 0)
    return v * abs(i) * 1e-12

groups = {u: unit_pids(u) for u in UNITS}
groups.update({k: [int(v)] for k, v in EXTRA.items()})
t0 = time.monotonic(); th0 = {g: threads(p) for g, p in groups.items()}; g0 = {g: gpu_ns(p) for g, p in groups.items()}
c0 = cpustat(); tr0 = trans()
where = {}; W = []; T = []; hot_n = 0; abort = False; nxt_w = nxt_t = nxt_c = t0; last = {}
while time.monotonic() - t0 < SECS:
    now = time.monotonic()
    if now >= nxt_c:
        nxt_c = now + 0.1
        cur = {}
        for g, th in th0.items():
            for tid, (comm, ns, pid) in th.items():
                ss = rd("/proc/%d/task/%d/schedstat" % (pid, tid)).split()
                if not ss: continue
                ns1 = int(ss[0]); d = ns1 - last.get(tid, ns)
                last[tid] = ns1
                if d > 0:
                    c = last_cpu(tid, pid); w = where.setdefault(tid, [0, 0])
                    w[1 if c >= 4 else 0] += d
    if now >= nxt_w:
        nxt_w = now + 0.2; W.append(watts())
    if now >= nxt_t:
        nxt_t = now + 1.0; t = temps(); T.append((now - t0, t))
        hot_n = hot_n + 1 if max(t.values()) > 95 else 0
        if hot_n >= 3:
            subprocess.run(["systemctl", "stop"] + UNITS); abort = True; break
    time.sleep(0.02)
el = time.monotonic() - t0
th1 = {g: threads(unit_pids(g) if g in UNITS else groups[g]) for g in groups}; g1 = {g: gpu_ns(unit_pids(g) if g in UNITS else groups[g]) for g in groups}
c1 = cpustat(); tr1 = trans()
res = {"name": NAME, "seconds": round(el, 1), "abort": abort, "units": {}}
for g in groups:
    by = {}
    for tid, (comm, ns, pid) in th1[g].items():
        d = ns - th0[g].get(tid, (comm, 0, pid))[1]
        w = where.get(tid, [0, 0]); tot = sum(w) or 1
        e = by.setdefault(comm, [0.0, 0.0])
        e[0] += d * w[0] / tot if sum(w) else d; e[1] += d * w[1] / tot
    thr = {k: [round(100 * v[0] / 1e9 / el, 1), round(100 * v[1] / 1e9 / el, 1)] for k, v in by.items() if (v[0] + v[1]) / 1e9 / el > 0.002}
    res["units"][g] = {"cpu_little_pct": round(sum(v[0] for v in thr.values()), 1), "cpu_big_pct": round(sum(v[1] for v in thr.values()), 1),
                       "gpu_pct": round(100 * (g1[g] - g0[g]) / 1e9 / el, 1), "threads": dict(sorted(thr.items(), key=lambda kv: -sum(kv[1])))}
res["cpu_little_pct"] = round(sum(u["cpu_little_pct"] for u in res["units"].values()), 1)
res["cpu_big_pct"] = round(sum(u["cpu_big_pct"] for u in res["units"].values()), 1)
res["gpu_pct"] = round(sum(u["gpu_pct"] for u in res["units"].values()), 1)
res["system_cpu_pct"] = {c: round(100 * (c1[c][0] - c0[c][0]) / max(1, c1[c][1] - c0[c][1]), 1) for c in sorted(c1) if c in c0}
tt = sum(tr1.get(f, 0) - tr0.get(f, 0) for f in tr1) or 1
res["gpu_mhz_share"] = {f: round(100 * (tr1[f] - tr0.get(f, 0)) / tt, 1) for f in sorted(tr1) if tr1[f] - tr0.get(f, 0) > 0}
res["watts_mean"] = round(sum(W) / max(1, len(W)), 2)
W60 = W[-300:]; res["watts_last60"] = round(sum(W60) / max(1, len(W60)), 2)
def agg(names, rows): return round(sum(max(t[n] for n in names if n in t) for _, t in rows) / max(1, len(rows)), 1)
last60 = [r for r in T if r[0] >= el - 60]
res["temp_last60"] = {"big": agg(BIG, last60), "little": agg(LITTLE, last60), "gpu": agg(GPU, last60), "hottest": round(sum(max(t.values()) for _, t in last60) / max(1, len(last60)), 1)}
res["temp_max"] = round(max(max(t.values()) for _, t in T), 1) if T else None
res["temp_start"] = round(max(T[0][1].values()), 1) if T else None
os.makedirs(OUT, exist_ok=True)
json.dump(res, open(os.path.join(OUT, NAME + ".json"), "w"), indent=1)
print("BUDGET", json.dumps(res))
