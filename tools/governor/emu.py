#!/usr/bin/env python3
"""emu.py NAME --ambient C --governor on|off [--seconds 420] [--ride DIR] [--out DIR] [--quota0 PCT]
          [--nominal road,signs,lights,env] [--road-hz N] [--tau S] [--max-level N]

Dev-box emulation of a platform at its thermal limit, with the real daemons (tools/governor/emu_stack.sh:
camerad-replay, modeld, signd with lights, envd, api, all on CPU backends) on a replayed ride. What is
emulated is the platform, not roadsense:

  * power: the reference phone's measured budget (docs/lanes/L21-budget-and-models.md) per unit of work,
    times the rates the stack is actually running at (from /v2/health):
        P = 1.62 W (idle, pins, camera, api) + 2.61 W * road/13.8 + 0.45 W * signs/5 + 1.55 W * lights/5
            + 0.65 W * env/1
  * temperature: first order, T -> ambient + 7.3 K/W * P with a time constant of --tau s (the phone's
    measured path to the air at fan 100 %; its real time constant is longer, the bench is shortened);
  * limiter: the kernel's passive trip at 90 C. Above it the CPU quota of the whole stack is cut by 4 % of
    its full value a second (down to 30 %), below 88 C given back by 2 % a second: every stage slows alike,
    which is what a clock cut does.

The governor (in the api) sees this platform through the file probe: one zone with limit 90 C and the limiter
state. --governor off sets "enabled": false: nothing is shed and the limiter is the only guard.
tools/governor/trace.py records what a consumer gets; compare the runs with tools/governor/compare.py.
"""
import json
import os
import shutil
import subprocess
import sys
import time
import urllib.request


def arg(flag, default=None):
    return sys.argv[sys.argv.index(flag) + 1] if flag in sys.argv else default


HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(os.path.dirname(HERE))
NAME = sys.argv[1]
AMBIENT = float(arg("--ambient", "22"))
GOV = arg("--governor", "on")
SECS = float(arg("--seconds", "420"))
RIDE = arg("--ride", os.path.expanduser("~/datasets/rides/krakow-2024"))
OUT = arg("--out", os.path.expanduser("~/datasets/l24gov/out"))
QUOTA0 = float(arg("--quota0", "0"))          # CPU quota at full clock, % of one core; 0 = unlimited (calibration)
TAU = float(arg("--tau", "40"))
PORT = int(arg("--port", "18190"))
MAXLVL = arg("--max-level")
K_PER_W, TRIP = 7.3, 90.0
# W per stage at its nominal rate; --nominal road,signs,lights,env gives the rates the emulated platform reaches
# at full clock (from a cool run), so that its full stack costs what the phone's does (6.9 W)
NOM = [float(x) for x in arg("--nominal", "13.8,5,5,1").split(",")]
W = {"base": 1.62, "road": (2.61, NOM[0]), "signs": (0.45, NOM[1]), "lights": (1.55, NOM[2]), "env": (0.65, NOM[3])}
UNIT = "l24emu"


def sh(*a):
    return subprocess.run(a, capture_output=True, text=True)


def health():
    try:
        return json.load(urllib.request.urlopen("http://127.0.0.1:%d/v2/health" % PORT, timeout=2))
    except Exception:
        return None


def main():
    os.makedirs(OUT, exist_ok=True)
    run = os.path.join(OUT, NAME + ".run")
    shutil.rmtree(run, ignore_errors=True)
    cfg = os.path.join(run, "cfg")
    os.makedirs(cfg)
    for f in ("camera-replay.json", "calib.json"):
        shutil.copy(os.path.join(RIDE, f), cfg)
    reading = os.path.join(run, "reading.json")
    # the emulated platform is slower than the phone: the "degraded" floors are 70 % of its own full-clock rates
    fl = {"hz": round(0.7 * NOM[0], 1), "latency_ms": 500.0}
    # the shed steps of governor.DEFAULT_STEPS with the rates scaled to this platform's full-clock rates
    steps = [{"what": "preview", "caps": {"preview_hz": 1}, "quick": True},
             {"what": "recorder_frames", "caps": {"record_frames": 0, "preview_hz": 0}, "quick": True},
             {"what": "scenery", "caps": {"env_hz": 0.2 * NOM[3]}},
             {"what": "view", "caps": {"view_hz": round(NOM[0] * 5 / 13.8, 2)}},
             {"what": "signs", "caps": {"signs_hz": NOM[1] / 2}},
             {"what": "lights", "caps": {"lights_hz": NOM[2] / 2}},
             {"what": "detectors_floor", "caps": {"env_hz": 0, "signs_hz": NOM[1] / 4, "lights_hz": NOM[2] / 4}},
             {"what": "plan", "caps": {"policy_hz": 2}, "quick": True},
             {"what": "road", "caps": {"road_hz": NOM[0] / 2}, "last_resort": True}]
    gcfg = {"probe": "file:" + reading, "enabled": GOV == "on", "steps": steps,
            "floors": {"road": fl, "lead": fl, "lanes": fl, "lights": {"hz": round(0.7 * NOM[2], 1), "latency_ms": 800.0},
                       "signs": {"hz": round(0.7 * NOM[1], 1), "latency_ms": 800.0}}}
    if MAXLVL is not None:
        gcfg["max_level"] = int(MAXLVL)
    json.dump(gcfg, open(os.path.join(cfg, "governor.json"), "w"))
    json.dump({"zones": [{"name": "soc", "temp_c": AMBIENT + 20, "limit_c": TRIP}], "limiter": []}, open(reading, "w"))
    mjpeg = [os.path.join(RIDE, f) for f in sorted(os.listdir(RIDE)) if f.endswith(".mjpeg")][0]
    env = {"RS_BUILD": os.environ.get("RS_BUILD", os.path.expanduser("~/datasets/l24gov/build-rs")),
           "SIGNS_BUILD": os.environ.get("SIGNS_BUILD", os.path.expanduser("~/datasets/l24gov/build-signs")),
           "RS_SRC": SRC, "MODELS": os.environ.get("MODELS", os.path.expanduser("~/projects/roadsense/models")),
           "SIGN_MODELS": os.environ.get("SIGN_MODELS", os.path.expanduser("~/projects/roadsense-signs/models")),
           "LIGHT_DET": os.environ.get("LIGHT_DET", os.path.expanduser("~/l7/models/bdd11n_320x960.onnx")),
           "LIGHT_STATE": os.environ.get("LIGHT_STATE", os.path.expanduser("~/l7/models/lights_state_bdd_48x24.onnx")),
           "PYTHON": sys.executable, "MODELD_HZ": arg("--road-hz", "14")}
    sh("systemctl", "--user", "stop", UNIT)
    sh("systemctl", "--user", "reset-failed", UNIT)
    cmd = ["systemd-run", "--user", "--unit", UNIT, "--collect", "-q", "-p", "MemoryMax=10G", "-p", "CPUWeight=10000"]
    if QUOTA0 > 0:
        cmd += ["-p", "CPUQuota=%d%%" % QUOTA0]
    for k, v in env.items():
        cmd.append("--setenv=%s=%s" % (k, v))
    cmd += ["sh", os.path.join(HERE, "emu_stack.sh"), run, cfg, mjpeg, str(PORT)]
    r = sh(*cmd)
    if r.returncode:
        sys.exit("cannot start the stack: " + r.stderr)
    try:
        for _ in range(60):
            if health():
                break
            time.sleep(0.5)
        else:
            sys.exit("api did not come up; see " + run)
        trace = subprocess.Popen([sys.executable, os.path.join(HERE, "trace.py"), os.path.join(OUT, NAME + ".jsonl"),
                                  str(SECS), "--url", "http://127.0.0.1:%d" % PORT, "--stop-c", "1000"])
        plant = open(os.path.join(OUT, NAME + ".plant.jsonl"), "w")
        temp, cut, t0, last = AMBIENT + 20.0, 0.0, time.time(), time.time()
        cpu0 = None
        while trace.poll() is None:
            time.sleep(1.0)
            now = time.time()
            dt, last = now - last, now
            h = health() or {}
            fps = h.get("fps") or {}
            p = W["base"] + sum(W[k][0] * (fps.get(k) or 0.0) / W[k][1] for k in ("road", "signs", "lights", "env"))
            temp += (AMBIENT + K_PER_W * p - temp) * min(1.0, dt / TAU)
            if temp > TRIP:
                cut = min(70.0, cut + 4.0 * dt)
            elif temp < TRIP - 2.0:
                cut = max(0.0, cut - 2.0 * dt)
            if QUOTA0 > 0:
                sh("systemctl", "--user", "set-property", "--runtime", UNIT + ".service",
                   "CPUQuota=%d%%" % max(1, round(QUOTA0 * (1 - cut / 100.0))))
            json.dump({"zones": [{"name": "soc", "temp_c": round(temp, 2), "limit_c": TRIP}],
                       "limiter": [{"name": "cpu-quota", "state": int(round(cut)), "max": 70}]}, open(reading + ".tmp", "w"))
            os.replace(reading + ".tmp", reading)
            u = sh("systemctl", "--user", "show", UNIT + ".service", "-p", "CPUUsageNSec", "--value").stdout.strip()
            cpu = int(u) / 1e9 if u.isdigit() else None
            cores = None if cpu is None or cpu0 is None else (cpu - cpu0) / dt
            cpu0 = cpu
            g = h.get("governor") or {}
            plant.write(json.dumps({"t": round(now - t0, 1), "temp": round(temp, 2), "p_w": round(p, 2), "cut_pct": round(cut, 1),
                                    "cores": None if cores is None else round(cores, 2), "level": g.get("level"),
                                    "fps": {k: fps.get(k) for k in ("road", "lead", "lanes", "signs", "lights", "env", "view", "camera")}}) + "\n")
            plant.flush()
    finally:
        sh("systemctl", "--user", "stop", UNIT)
    print("done", NAME)


main()
