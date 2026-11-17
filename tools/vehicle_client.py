#!/usr/bin/env python3
"""Reference client of the roadsense vehicle input API (docs/VEHICLE_API.md).

Copy this class into a feeder; it has no dependency beyond the standard library and imports nothing from
roadsense. It connects to <rundir>/position.sock, reconnects on its own, never blocks the caller (messages are
dropped while mapd is away), stamps every message with CLOCK_MONOTONIC and reads mapd's hello reply, so the
feeder knows what this roadsense accepts (an older one takes `gnss`, `motion` and `reset` only).

    feed = VehicleFeed("/run/roadsense", name="my-car",
                       provides={"motion": ["speed_mps", "yaw_rate_rps"], "controls": ["brake", "turn_signal"]})
    feed.describe(name="My car", wheelbase_m=2.6, track_m=1.5, camera={"x_m": 1.7, "y_m": 0.0, "z_m": 1.25})
    feed.gnss(lat=52.2297, lon=21.0122, hacc_m=4.0, speed_mps=13.9, course_deg=271.5, source="phone")
    feed.motion(speed_mps=13.8, yaw_rate_rps=0.02, gear="D", quality={"yaw_rate_rps": "uncalibrated"}, source="can")
    feed.controls(turn_signal="left", brake=False, source="can")
    feed.body(low_beam=True, wipers="off", outside_temp_c=7.5, source="can")
    feed.objects("front_radar", [{"id": 4, "range_m": 31.2, "azimuth_deg": -1.0, "range_rate_mps": -0.8}], kind="radar")
    feed.no_fix(source="phone")               # the receiver reports that it lost the fix
    feed.reset("test drive ended")            # forget the position (map: no_source until the next fix)

`controls`, `body`, `describe` and `objects` return False without sending while the connected roadsense does not
list the message type in its hello reply (`feed.supports("controls")`); `gnss`, `motion` and `reset` are sent to
any version. `describe` is remembered and sent again after every reconnect.

Command line (one message, or a stream of JSON lines from stdin):

    tools/vehicle_client.py --rundir /tmp/roadsense gnss 52.2297 21.0122 --hacc 4 --speed 13.9 --course 271.5
    tools/vehicle_client.py --rundir /tmp/roadsense controls --turn-signal left --brake 1
    tools/vehicle_client.py --rundir /tmp/roadsense hello          # print what this roadsense accepts
    some-source | tools/vehicle_client.py --rundir /tmp/roadsense stdin
"""
import argparse
import json
import os
import socket
import sys
import time

API = 1
V1_TYPES = ("gnss", "motion", "reset")       # what a roadsense without a hello reply (or an old one) accepts


class VehicleFeed:
    def __init__(self, rundir="/run/roadsense", name="feeder", provides=None, retry_s=1.0):
        self.path = os.path.join(rundir, "position.sock")
        self.name, self.provides, self.retry_s = name, provides, retry_s
        self.sock = None
        self._next_try = 0.0
        self._buf = b""
        self._description = None
        self._described = False
        self.sent = self.dropped = self.connects = self.errors_total = 0
        self.errors = []                 # error replies from mapd (most recent last, at most 20)
        self.server = None               # mapd's hello reply on this connection (None until it was read)
        self.accepts = None              # its "accepts" list

    @property
    def connected(self):
        return self.sock is not None

    def supports(self, mtype):
        """Does the connected roadsense take this message type? The v1 types always; the others once its hello
        reply lists them."""
        return mtype in V1_TYPES or (self.accepts is not None and mtype in self.accepts)

    def _connect(self):
        now = time.monotonic()
        if now < self._next_try:
            return False
        self._next_try = now + self.retry_s
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        hello = {"type": "hello", "api": API, "name": self.name}
        if self.provides:
            hello["provides"] = self.provides
        try:
            s.connect(self.path)
            s.setblocking(False)
            s.send((json.dumps(hello) + "\n").encode())
        except OSError:
            s.close()
            return False
        self.sock, self._buf = s, b""
        self.server = self.accepts = None
        self._described = False
        self.connects += 1
        return True

    def poll(self):
        """Read pending replies (hello, errors) without sending anything."""
        if self.sock is None:
            return
        try:
            data = self.sock.recv(65536)
        except BlockingIOError:
            return
        except OSError:
            data = b""
        if not data:
            self.close()
            return
        self._buf += data
        *lines, self._buf = self._buf.split(b"\n")
        for line in lines:
            try:
                m = json.loads(line)
            except ValueError:
                continue
            if m.get("type") == "hello":
                self.server = m
                acc = m.get("accepts")
                self.accepts = [str(a) for a in acc] if isinstance(acc, list) else []
            elif m.get("type") == "error":
                self.errors = (self.errors + [m])[-20:]
                self.errors_total += 1

    def send(self, msg):
        """Send one message (a dict with a "type"). Returns False when it was dropped."""
        if "ts_mono_ns" not in msg and "age_ms" not in msg:
            msg["ts_mono_ns"] = time.monotonic_ns()
        if self.sock is None and not self._connect():
            self.dropped += 1
            return False
        data = (json.dumps(msg, separators=(",", ":")) + "\n").encode()
        try:
            n = self.sock.send(data)
        except BlockingIOError:              # mapd is not reading: drop rather than block the car
            self.dropped += 1
            return False
        except OSError:
            self.close()
            self.dropped += 1
            return False
        if n != len(data):                   # half a line in the stream: start over on a new connection
            self.close()
            self.dropped += 1
            return False
        self.sent += 1
        self.poll()
        if self._description is not None and not self._described and self.accepts is not None:
            self._described = True           # once per connection, as soon as the hello reply is in
            if "vehicle" in self.accepts:
                self.send(dict(self._description))
        return True

    def _optional(self, mtype, fields, ts_mono_ns):
        if not self.supports(mtype):
            if self.sock is None:
                self._connect()              # so that the hello reply gets read at all
            self.poll()
            if not self.supports(mtype):
                return False
        m = {"type": mtype}
        m.update({k: v for k, v in fields.items() if v is not None})
        if ts_mono_ns is not None:
            m["ts_mono_ns"] = ts_mono_ns
        return self.send(m)

    # ---- position (v1) ----
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

    def reset(self, reason=None, ts_mono_ns=None):
        """Make mapd forget the position (filter, fix, match): the map is `no_source` until a new fix arrives."""
        m = {"type": "reset"}
        if reason is not None:
            m["reason"] = str(reason)
        if ts_mono_ns is not None:
            m["ts_mono_ns"] = ts_mono_ns
        return self.send(m)

    def motion(self, speed_mps=None, ts_mono_ns=None, **signals):
        """The car's own motion: speed_mps, gear, reverse, yaw_rate_rps, wheel_speeds_mps, steering_wheel_deg,
        accel_long_mps2, accel_lat_mps2, plus quality={signal: ...} and source="tag"."""
        m = {"type": "motion"}
        if speed_mps is not None:
            m["speed_mps"] = speed_mps
        m.update({k: v for k, v in signals.items() if v is not None})
        if ts_mono_ns is not None:
            m["ts_mono_ns"] = ts_mono_ns
        return self.send(m)

    # ---- the rest of the car (since 1.1; not sent to a roadsense that does not accept them) ----
    def controls(self, ts_mono_ns=None, **signals):
        """turn_signal, brake, brake_pressure, accelerator, cruise, cruise_set_speed_mps, parking_brake."""
        return self._optional("controls", signals, ts_mono_ns)

    def body(self, ts_mono_ns=None, **signals):
        """low_beam, high_beam, parking_lights, fog_front, fog_rear, wipers, rain, ambient_light_lux,
        outside_temp_c, door_open."""
        return self._optional("body", signals, ts_mono_ns)

    def objects(self, sensor, objects, kind=None, tracked=None, ts_mono_ns=None, **extra):
        """One scan of a range sensor: every object it reports now (an empty list = it sees nothing)."""
        return self._optional("objects", dict(extra, sensor=sensor, objects=objects, kind=kind, tracked=tracked),
                              ts_mono_ns)

    def describe(self, **description):
        """The static description of the car (wheelbase_m, track_m, steering_ratio, camera, sensors, ...). Sent
        now when possible and again after every reconnect."""
        self._description = dict(description, type="vehicle")
        self._described = False
        if self.sock is None:
            self._connect()
        self.poll()
        if self.accepts is not None:
            self._described = True
            if "vehicle" in self.accepts:
                return self.send(dict(self._description))
        return False

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None


def _flag(v):
    return {"1": True, "true": True, "on": True, "0": False, "false": False, "off": False}[v.lower()]


def main():
    ap = argparse.ArgumentParser(description="send vehicle input to roadsense (mapd)")
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
    r = sub.add_parser("reset", help="make mapd forget the position")
    r.add_argument("--reason")
    m = sub.add_parser("motion")
    m.add_argument("speed", type=float)
    m.add_argument("--yaw-rate", type=float)
    m.add_argument("--steering", type=float)
    m.add_argument("--gear", choices=list("PRNDB"))
    c = sub.add_parser("controls")
    c.add_argument("--turn-signal", choices=("off", "left", "right", "both"))
    c.add_argument("--brake", type=_flag)
    c.add_argument("--accelerator", type=float)
    c.add_argument("--cruise", choices=("off", "standby", "active"))
    c.add_argument("--cruise-set-speed", type=float, help="m/s")
    b = sub.add_parser("body")
    b.add_argument("--low-beam", type=_flag)
    b.add_argument("--high-beam", type=_flag)
    b.add_argument("--wipers", choices=("off", "intermittent", "low", "high"))
    b.add_argument("--outside-temp", type=float)
    sub.add_parser("hello", help="print the hello reply (what this roadsense accepts)")
    sub.add_parser("stdin", help="forward JSON lines from stdin (ts_mono_ns added when missing)")
    a = ap.parse_args()
    feed = VehicleFeed(a.rundir, name="vehicle_client")

    def wait_hello():
        feed._connect()
        end = time.monotonic() + 1.0
        while feed.accepts is None and feed.sock is not None and time.monotonic() < end:
            time.sleep(0.02)
            feed.poll()

    ok = True
    if a.cmd == "gnss":
        for i in range(a.repeat):
            ok = feed.gnss(a.lat, a.lon, a.hacc, a.speed, a.course, source=a.source)
            if i + 1 < a.repeat:
                time.sleep(1.0)
    elif a.cmd == "nofix":
        ok = feed.no_fix(source="cli")
    elif a.cmd == "reset":
        ok = feed.reset(a.reason)
    elif a.cmd == "motion":
        ok = feed.motion(a.speed, yaw_rate_rps=a.yaw_rate, steering_wheel_deg=a.steering, gear=a.gear, source="cli")
    elif a.cmd == "controls":
        wait_hello()
        ok = feed.controls(turn_signal=a.turn_signal, brake=a.brake, accelerator=a.accelerator, cruise=a.cruise,
                           cruise_set_speed_mps=a.cruise_set_speed, source="cli")
    elif a.cmd == "body":
        wait_hello()
        ok = feed.body(low_beam=a.low_beam, high_beam=a.high_beam, wipers=a.wipers, outside_temp_c=a.outside_temp,
                       source="cli")
    elif a.cmd == "hello":
        wait_hello()
        ok = feed.server is not None
        print(json.dumps(feed.server, indent=1) if ok else "no reply (mapd not running, or older than the hello message)")
    else:
        for line in sys.stdin:
            line = line.strip()
            if line:
                ok = feed.send(json.loads(line)) and ok
    time.sleep(0.1)
    feed.poll()
    for e in feed.errors:
        print("mapd:", e.get("reason"), file=sys.stderr)
    if not ok and not feed.errors and a.cmd in ("controls", "body"):
        print("not sent: this roadsense does not accept `%s` (or mapd is not running)" % a.cmd, file=sys.stderr)
    sys.exit(0 if ok and not feed.errors else 1)


if __name__ == "__main__":
    main()
