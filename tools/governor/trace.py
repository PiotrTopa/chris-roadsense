#!/usr/bin/env python3
"""trace.py OUT.jsonl SECONDS [--url http://127.0.0.1:8090] [--stop-cmd CMD] [--stop-c 95] [--stop-n 3]   (on the device)

What a consumer gets, and what the platform does meanwhile, in one file of JSON lines:

  {"k":"road", "t":s since start, "seq":frame, "lat":ms camera->here, "model_ms", "rate_hz", "cap", "lvl",
   "ly":[left y at 9/32/61 m], "ry":[right ...], "lp":[left prob, right prob], "ld":[lead prob, x]}
  {"k":"lead", "seq", "lat", "present", "mode", "dist", "vrel", "rate":{...}}
  {"k":"lanes","seq", "lat", "l":[present, y9, y32], "r":[...], "width", "rate":{...}}
  {"k":"lights"|"signs"|"env"|"view"|"camera", "seq", "lat", "n":objects, "rate":{...}}
  {"k":"sys", "t", "zones":{name: C}, "cool":{name: state}, "freq":{policy: kHz, "gpu": Hz}, "w":rail W,
   "gov":{level, shed, headroom_k, limited, topics}}                                  once a second

Safety: when the hottest zone stays above --stop-c for --stop-n samples (1 a second; default 3), --stop-cmd is run and the trace ends
with {"k":"abort"}.
"""
import asyncio
import glob
import json
import subprocess
import sys
import time

import aiohttp


def arg(flag, default=None):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


OUT, SECS = sys.argv[1], float(sys.argv[2])
URL = arg("--url", "http://127.0.0.1:8090")
STOP_CMD, STOP_C = arg("--stop-cmd"), float(arg("--stop-c", "95"))
STOP_N = int(arg("--stop-n", "3"))
IDX = (7, 13, 18)          # X_IDXS 9.2, 31.7, 60.8 m


def rd(p):
    try:
        with open(p) as f:
            return f.read().strip()
    except OSError:
        return ""


ZONES = {rd(z + "/type").replace("-thermal", ""): z + "/temp" for z in glob.glob("/sys/class/thermal/thermal_zone*")}
COOL = {rd(c + "/type"): c + "/cur_state" for c in glob.glob("/sys/class/thermal/cooling_device*")}
POL = {p.rsplit("/", 1)[1]: p + "/scaling_cur_freq" for p in glob.glob("/sys/devices/system/cpu/cpufreq/policy*")}
GPUF = (glob.glob("/sys/class/devfreq/*gpu*/cur_freq") or [None])[0]
BAT = (glob.glob("/sys/class/power_supply/*battery*") or glob.glob("/sys/class/power_supply/*") or [None])[0]


def num(s, scale=1.0):
    try:
        return float(s) * scale
    except ValueError:
        return None


def rail_w():
    if not BAT:
        return None
    v, i = num(rd(BAT + "/voltage_now"), 1e-6), num(rd(BAT + "/current_now"), 1e-6)
    return None if v is None or i is None else round(v * abs(i), 3)


def pick(line, idx=IDX):
    y = (line or {}).get("y")
    return [round(y[i], 3) for i in idx] if isinstance(y, list) and len(y) > max(idx) else None


async def main():
    t0 = time.time()
    f = open(OUT, "w")

    def put(d):
        f.write(json.dumps(d, separators=(",", ":")) + "\n")

    async def sys_loop(session):
        hot_n = 0
        while True:
            zones = {n: num(rd(p), 1e-3) for n, p in ZONES.items()}
            w = []
            for _ in range(5):
                w.append(rail_w())
                await asyncio.sleep(0.2)
            w = [x for x in w if x is not None]
            gov = None
            try:
                async with session.get(URL + "/v2/health", timeout=aiohttp.ClientTimeout(total=2)) as r:
                    h = await r.json()
                    g = h.get("governor") or {}
                    gov = {k: g.get(k) for k in ("level", "shed", "headroom_k", "limited", "hottest", "topics", "forced")}
                    gov["fps"] = h.get("fps")
            except Exception:
                pass
            put({"k": "sys", "t": round(time.time() - t0, 2), "zones": zones,
                 "cool": {n: num(rd(p)) for n, p in COOL.items()},
                 "freq": dict({n: num(rd(p)) for n, p in POL.items()}, gpu=num(rd(GPUF)) if GPUF else None),
                 "w": round(sum(w) / len(w), 3) if w else None, "gov": gov})
            f.flush()
            hot = max([v for v in zones.values() if v is not None and v < 150] or [0])
            hot_n = hot_n + 1 if hot > STOP_C else 0
            if hot_n >= STOP_N:
                put({"k": "abort", "t": round(time.time() - t0, 2), "hot": hot})
                f.flush()
                if STOP_CMD:
                    subprocess.run(STOP_CMD, shell=True)
                return

    async with aiohttp.ClientSession() as session:
        st = asyncio.create_task(sys_loop(session))
        async with session.ws_connect(URL + "/ws", max_msg_size=0) as ws:
            while time.time() - t0 < SECS and not st.done():
                try:
                    m = await asyncio.wait_for(ws.receive(), timeout=1.0)
                except asyncio.TimeoutError:
                    continue
                if m.type != aiohttp.WSMsgType.TEXT:
                    if m.type in (aiohttp.WSMsgType.CLOSED, aiohttp.WSMsgType.ERROR, aiohttp.WSMsgType.CLOSE):
                        break
                    continue
                now = time.time_ns()
                j = json.loads(m.data)
                k = j.get("type")
                ts = j.get("ts_real_ns")
                d = {"k": k, "t": round(now / 1e9 - t0, 3), "seq": j.get("seq") or j.get("frame_seq"),
                     "lat": round((now - ts) / 1e6, 1) if ts else None}
                if k == "road":
                    ll = j.get("lane_lines") or [{}] * 4
                    ld = (j.get("leads") or [{}])[0]
                    d.update(model_ms=j.get("model_ms"), rate_hz=j.get("rate_hz"), cap=j.get("effective_hz"),
                             lvl=j.get("governor_level"), ly=pick(ll[1]), ry=pick(ll[2]),
                             lp=[ll[1].get("prob"), ll[2].get("prob")],
                             ld=[ld.get("prob"), (ld.get("x") or [None])[0]])
                elif k == "lead":
                    d.update(present=j.get("present"), mode=j.get("mode"), dist=j.get("dist_m"), vrel=j.get("v_rel_mps"),
                             rate=j.get("rate"))
                elif k == "lanes":
                    ls = j.get("lines") or [{}] * 4
                    d.update(l=[ls[1].get("present")] + (pick(ls[1], IDX[:2]) or [None, None]),
                             r=[ls[2].get("present")] + (pick(ls[2], IDX[:2]) or [None, None]),
                             width=j.get("lane_width_m"), rate=j.get("rate"))
                elif k in ("lights", "signs"):
                    d.update(n=len(j.get("lights") or j.get("signs") or j.get("tracks") or []), rate=j.get("rate"))
                elif k in ("env", "view", "camera"):
                    pass
                else:
                    continue
                put(d)
        st.cancel()
    put({"k": "end", "t": round(time.time() - t0, 2)})
    f.close()


asyncio.run(main())
