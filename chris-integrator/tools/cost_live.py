#!/usr/bin/env python3
"""What a running chris-integrator costs: CPU of the process against what the mixer accounts for.

    tools/cost_live.py [--seconds 240] [--url http://127.0.0.1:8090] [--unit chris-integrator.service] [--json OUT]

Reads /proc/<pid>/stat (user + system time, all threads), /proc/<pid>/status (resident memory), the thermal
zones and the little cluster's frequency table at the start and the end of the window, and `/v2/mixer` +
`/v2/health` of the daemon (two requests at each end; none in between, so the window holds no polling of its
own). Prints the CPU share of one core, CPU per road frame, the plug-ins' own time per frame, the part no
plug-in accounts for (reading, parsing, output channels, the event loop), latencies and temperatures.
Standard library only; runs on the device (no root needed)."""
import argparse
import glob
import json
import os
import subprocess
import time
import urllib.request


def get(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return json.loads(r.read())


def proc_cpu(pid):
    with open("/proc/%d/stat" % pid) as f:
        p = f.read().rsplit(")", 1)[1].split()
    tck = os.sysconf("SC_CLK_TCK")
    return int(p[11]) / tck, int(p[12]) / tck          # utime, stime


def rss_mb(pid):
    with open("/proc/%d/status" % pid) as f:
        for ln in f:
            if ln.startswith("VmRSS"):
                return int(ln.split()[1]) / 1024.0
    return None


def temps():
    out = {}
    for d in glob.glob("/sys/class/thermal/thermal_zone*"):
        try:
            with open(d + "/temp") as f:
                t = int(f.read()) / 1000.0
            with open(d + "/type") as f:
                out[f.read().strip()] = t
        except (OSError, ValueError):
            pass
    return out


def freq_table(cpu=0):
    try:
        with open("/sys/devices/system/cpu/cpu%d/cpufreq/stats/time_in_state" % cpu) as f:
            return {int(a): int(b) for a, b in (ln.split() for ln in f)}
    except (OSError, ValueError):
        return {}


def snap(a, pid):
    return {"t": time.monotonic(), "cpu": proc_cpu(pid), "rss": rss_mb(pid), "mixer": get(a.url + "/v2/mixer"),
            "health": get(a.url + "/v2/health"), "freq": freq_table()}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=240)
    ap.add_argument("--url", default="http://127.0.0.1:8090")
    ap.add_argument("--unit", default="chris-integrator.service")
    ap.add_argument("--pid", type=int)
    ap.add_argument("--json")
    a = ap.parse_args()
    pid = a.pid or int(subprocess.check_output(["systemctl", "show", "-p", "MainPID", "--value", a.unit]).strip())
    s0 = snap(a, pid)
    peak, hot = {}, 0.0
    end = time.monotonic() + a.seconds
    while time.monotonic() < end:
        time.sleep(min(5.0, max(0.1, end - time.monotonic())))
        for k, v in temps().items():
            if v > peak.get(k, -1e9):
                peak[k] = v
        hot = max(peak.values()) if peak else 0.0
        if hot > 95.0:
            print("STOP: %.1f C" % hot)
            break
    s1 = snap(a, pid)
    dt = s1["t"] - s0["t"]
    u, s = s1["cpu"][0] - s0["cpu"][0], s1["cpu"][1] - s0["cpu"][1]
    st0, st1 = s0["mixer"]["streams"], s1["mixer"]["streams"]
    n = {k: st1[k]["messages"] - st0.get(k, {}).get("messages", 0) for k in st1}
    road = n.get("road", 0)
    p0 = {p["name"]: p for p in s0["mixer"]["plugins"]}
    rows, acc = [], 0.0
    for p in s1["mixer"]["plugins"]:
        q = p0.get(p["name"], {"cpu_s": 0.0, "calls": 0})
        c, k = p["cpu_s"] - q["cpu_s"], p["calls"] - q["calls"]
        acc += c
        rows.append((p["name"], k, c, 1e3 * c / k if k else 0.0, p["max_ms"]))
    f0, f1 = s0["freq"], s1["freq"]
    fr = {k: f1[k] - f0.get(k, 0) for k in f1 if f1[k] - f0.get(k, 0) > 0}
    tot = sum(fr.values()) or 1
    res = {"seconds": round(dt, 1), "pid": pid, "cpu_user_s": round(u, 2), "cpu_sys_s": round(s, 2),
           "cpu_share_pct": round(100 * (u + s) / dt, 1), "road_frames": road, "road_hz": round(road / dt, 2),
           "cpu_ms_per_road_frame": round(1e3 * (u + s) / road, 2) if road else None,
           "plugins_ms_per_road_frame": round(1e3 * acc / road, 2) if road else None,
           "unaccounted_ms_per_road_frame": round(1e3 * (u + s - acc) / road, 2) if road else None,
           "rss_mb": [round(s0["rss"], 1), round(s1["rss"], 1)],
           "messages_per_s": {k: round(v / dt, 2) for k, v in sorted(n.items()) if v},
           "plugins": [{"name": r[0], "calls": r[1], "cpu_s": round(r[2], 2), "ms_per_call": round(r[3], 3),
                        "ms_per_road_frame": round(1e3 * r[2] / road, 3) if road else None, "max_ms_since_start": r[4]} for r in rows],
           "latency_ms": {t: (v or {}).get("latency_ms") for t, v in ((s1["health"].get("governor") or {}).get("topics") or {}).items()},
           "governor_level": (s1["health"].get("governor") or {}).get("level"),
           "peak_temp_c": round(hot, 1), "hottest": sorted(peak.items(), key=lambda kv: -kv[1])[:3],
           "cpu0_freq_share": {str(k): round(v / tot, 3) for k, v in sorted(fr.items())}}
    if a.json:
        with open(a.json, "w") as f:
            json.dump(res, f, indent=1)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
