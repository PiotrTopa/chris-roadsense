#!/usr/bin/env python3
"""A minimal feeder for "any car": OBD-II vehicle speed + a GNSS receiver that speaks NMEA.

    tools/examples/obd_gps_feeder.py --nmea /dev/ttyACM0 --obd /dev/rfcomm0
    tools/examples/obd_gps_feeder.py --demo          # no hardware: a made-up drive, tagged "synthetic"

What it shows (docs/VEHICLE_API.md):

* the whole contract of a small feeder is two message types: `gnss` once a second and `motion` a few
  times a second. Everything else is optional;
* the hello handshake: the feeder says what it will send, roadsense says what it accepts;
* honest quality: an OBD-II speed (PID 0x0D) is whole km/h, so it is sent as `uncalibrated` and roadsense
  estimates its scale against GNSS; no yaw signal at all is fine, the heading then comes from the fixes and
  the map;
* timestamps: every sample is stamped when it was read (the client adds CLOCK_MONOTONIC), never later.

Needs `tools/vehicle_client.py` next to it (copy both), and python-OBD (`pip install obd`) for --obd.
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from vehicle_client import VehicleFeed      # noqa: E402


def nmea_fixes(path):
    """Yield fixes from GGA + RMC sentences of a serial NMEA receiver (blocking read, 1 line at a time)."""
    fix = {}
    with open(path, "r", errors="replace") as f:
        for line in f:
            p = line.strip().split("*")[0].split(",")
            if len(p[0]) < 6:
                continue
            kind = p[0][3:6]
            try:
                if kind == "GGA" and len(p) > 8:
                    if p[6] in ("", "0"):
                        yield {"fix": "none"}
                        continue
                    lat = (int(p[2][:2]) + float(p[2][2:]) / 60.0) * (-1 if p[3] == "S" else 1)
                    lon = (int(p[4][:3]) + float(p[4][3:]) / 60.0) * (-1 if p[5] == "W" else 1)
                    fix = {"lat": lat, "lon": lon, "sats": int(p[7] or 0), "hdop": float(p[8] or 0),
                           "hacc_m": max(2.0, 4.0 * float(p[8] or 2.5)),       # no accuracy in NMEA: HDOP x 4 m
                           "fix": "dgps" if p[6] == "2" else "3d"}
                elif kind == "RMC" and len(p) > 8 and p[2] == "A" and fix:
                    if p[7]:
                        fix["speed_mps"] = float(p[7]) * 0.514444
                    if p[8] and fix.get("speed_mps", 0.0) > 1.0:
                        fix["course_deg"] = float(p[8])
                    yield fix
                    fix = {}
            except ValueError:
                continue


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--rundir", default=os.environ.get("ROADSENSE_RUNDIR", "/run/roadsense"))
    ap.add_argument("--nmea", help="serial device of the GNSS receiver")
    ap.add_argument("--obd", help="ELM327 port for python-OBD")
    ap.add_argument("--demo", action="store_true", help="a made-up drive in a circle (no hardware)")
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0 = run until stopped)")
    a = ap.parse_args()
    feed = VehicleFeed(a.rundir, name="obd-gps-example",
                       provides={"gnss": ["lat", "lon", "hacc_m", "speed_mps", "course_deg"], "motion": ["speed_mps"]})
    end = time.monotonic() + a.seconds if a.seconds else None
    if a.demo:
        feed.describe(name="Demo car", wheelbase_m=2.6, track_m=1.5)
        t0 = time.monotonic()
        nxt_fix = 0.0
        while end is None or time.monotonic() < end:
            t = time.monotonic() - t0
            ang, r, v = t * 12.0 / 300.0, 300.0, 12.0             # 12 m/s on a 300 m circle around a point in Warsaw
            feed.motion(speed_mps=v, yaw_rate_rps=v / r, gear="D", source="synthetic")
            if t >= nxt_fix:
                nxt_fix = t + 1.0
                lat = 52.2297 + r * math.sin(ang) / 111320.0
                lon = 21.0122 + r * math.cos(ang) / (111320.0 * math.cos(math.radians(52.2297)))
                feed.gnss(lat, lon, hacc_m=4.0, speed_mps=v, course_deg=(-math.degrees(ang)) % 360.0, source="synthetic")
            time.sleep(0.1)
        return
    if not a.nmea:
        ap.error("--nmea DEVICE (and optionally --obd PORT), or --demo")
    speed = None
    if a.obd:
        import obd                                               # python-OBD
        conn = obd.Async(a.obd)
        conn.watch(obd.commands.SPEED)
        conn.start()
        speed = lambda: conn.query(obd.commands.SPEED)            # noqa: E731
    import threading

    def motion():
        while end is None or time.monotonic() < end:
            r = speed()
            if not r.is_null():
                feed.motion(speed_mps=r.value.to("m/s").magnitude, quality={"speed_mps": "uncalibrated"}, source="obd2")
            time.sleep(0.2)
    if speed is not None:
        threading.Thread(target=motion, daemon=True).start()
    for fix in nmea_fixes(a.nmea):
        if fix.get("fix") == "none":
            feed.no_fix(source="nmea")
        else:
            feed.gnss(fix.pop("lat"), fix.pop("lon"), source="nmea", **fix)
        if end is not None and time.monotonic() > end:
            break


if __name__ == "__main__":
    main()
