#!/usr/bin/env python3
"""Reference client of the roadsense position input API v1 (docs/ARCHITECTURE.md "Position input").

Copy this class into a feeder; it has no dependency beyond the standard library and imports nothing
from roadsense. It connects to <rundir>/position.sock, reconnects on its own, never blocks the caller
(messages are dropped while mapd is away) and stamps every message with CLOCK_MONOTONIC.

    feed = PositionFeed("/run/roadsense")
    feed.gnss(lat=52.2297, lon=21.0122, hacc_m=4.0, speed_mps=13.9, course_deg=271.5, sats=14, source="headunit")
    feed.motion(speed_mps=13.8, steering_wheel_deg=-2.5, gear="D", source="prius-can")
    feed.no_fix(source="headunit")            # the receiver reports that it lost the fix

Command line (one message, or a stream of JSON lines from stdin):

    tools/position_client.py --rundir /tmp/roadsense gnss 52.2297 21.0122 --hacc 4 --speed 13.9 --course 271.5
    some-source | tools/position_client.py --rundir /tmp/roadsense stdin
"""
import argparse
import json
import os
import socket
import sys
import time

API = 1


class PositionFeed:
    def __init__(self, rundir="/run/roadsense", name="feeder", retry_s=1.0):
        self.path = os.path.join(rundir, "position.sock")
        self.name, self.retry_s = name, retry_s
        self.sock = None
        self._next_try = 0.0
        self.sent = self.dropped = 0
        self.errors = []                 # error replies from mapd (most recent last, at most 20)

    def _connect(self):
        now = time.monotonic()
        if now < self._next_try:
            return False
        self._next_try = now + self.retry_s
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            s.connect(self.path)
            s.setblocking(False)
            s.send((json.dumps({"type": "hello", "api": API, "name": self.name}) + "\n").encode())
        except OSError:
            s.close()
            return False
        self.sock = s
        return True

    def _poll_replies(self):
        try:
            data = self.sock.recv(65536)
        except BlockingIOError:
            return
        except OSError:
            data = b""
        if not data:
            self.close()
            return
        for line in data.splitlines():
            try:
                m = json.loads(line)
            except ValueError:
                continue
            if m.get("type") == "error":
                self.errors = (self.errors + [m])[-20:]

    def send(self, msg):
        """Send one message (a dict with "type": "gnss" or "motion"). Returns False when it was dropped."""
        if "ts_mono_ns" not in msg:
            msg["ts_mono_ns"] = time.monotonic_ns()
        if self.sock is None and not self._connect():
            self.dropped += 1
            return False
        try:
            self.sock.send((json.dumps(msg, separators=(",", ":")) + "\n").encode())
        except BlockingIOError:              # mapd is not reading: drop rather than block the car
            self.dropped += 1
            return False
        except OSError:
            self.close()
            self.dropped += 1
            return False
        self.sent += 1
        self._poll_replies()
        return True

    def gnss(self, lat, lon, hacc_m=None, speed_mps=None, course_deg=None, fix="3d", ts_mono_ns=None, **extra):
        m = {"type": "gnss", "fix": fix, "lat": lat, "lon": lon}
        for k, v in (("hacc_m", hacc_m), ("speed_mps", speed_mps), ("course_deg", course_deg),
                     ("ts_mono_ns", ts_mono_ns)):
            if v is not None:
                m[k] = v
        m.update(extra)
        return self.send(m)

    def no_fix(self, ts_mono_ns=None, **extra):
        m = {"type": "gnss", "fix": "none"}
        if ts_mono_ns is not None:
            m["ts_mono_ns"] = ts_mono_ns
        m.update(extra)
        return self.send(m)

    def motion(self, speed_mps, yaw_rate_rps=None, steering_wheel_deg=None, wheel_speeds_mps=None, gear=None,
               reverse=None, ts_mono_ns=None, **extra):
        m = {"type": "motion", "speed_mps": speed_mps}
        for k, v in (("yaw_rate_rps", yaw_rate_rps), ("steering_wheel_deg", steering_wheel_deg),
                     ("wheel_speeds_mps", wheel_speeds_mps), ("gear", gear), ("reverse", reverse),
                     ("ts_mono_ns", ts_mono_ns)):
            if v is not None:
                m[k] = v
        m.update(extra)
        return self.send(m)

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None


def main():
    ap = argparse.ArgumentParser(description="send position input to roadsense mapd")
    ap.add_argument("--rundir", default=os.environ.get("ROADSENSE_RUNDIR", "/run/roadsense"))
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("gnss")
    g.add_argument("lat", type=float)
    g.add_argument("lon", type=float)
    g.add_argument("--hacc", type=float)
    g.add_argument("--speed", type=float)
    g.add_argument("--course", type=float)
    g.add_argument("--source", default="cli")
    g.add_argument("--repeat", type=int, default=1, help="send N times, one per second")
    sub.add_parser("nofix")
    m = sub.add_parser("motion")
    m.add_argument("speed", type=float)
    m.add_argument("--yaw-rate", type=float)
    m.add_argument("--steering", type=float)
    sub.add_parser("stdin", help="forward JSON lines from stdin (ts_mono_ns added when missing)")
    a = ap.parse_args()
    feed = PositionFeed(a.rundir, name="position_client")
    if a.cmd == "gnss":
        for i in range(a.repeat):
            ok = feed.gnss(a.lat, a.lon, a.hacc, a.speed, a.course, source=a.source)
            if i + 1 < a.repeat:
                time.sleep(1.0)
    elif a.cmd == "nofix":
        ok = feed.no_fix(source="cli")
    elif a.cmd == "motion":
        ok = feed.motion(a.speed, yaw_rate_rps=a.yaw_rate, steering_wheel_deg=a.steering, source="cli")
    else:
        ok = True
        for line in sys.stdin:
            line = line.strip()
            if line:
                ok = feed.send(json.loads(line)) and ok
    time.sleep(0.1)
    if feed.sock is not None:
        feed._poll_replies()
    for e in feed.errors:
        print("mapd:", e.get("reason"), file=sys.stderr)
    sys.exit(0 if ok and not feed.errors else 1)


if __name__ == "__main__":
    main()
