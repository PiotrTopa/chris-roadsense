"""The plug-in contract out of process: the same declarations and the same calls, as JSON lines over a pipe
(a child process' stdin / stdout) or a Unix socket (a separately run daemon).

    plug-in -> mixer   {"type":"plugin","contract":1,"name":..,"inputs":[{"name","trigger","optional","feedback"}],
                        "outputs":[{"name","kind","time","public","doc"}]}                       once, first
    mixer -> plug-in   {"type":"config","cfg":{...}}                                               once, optional
                       {"in":"<stream>","msg":{...},"n":<int>}       a message of a declared input; `n` is set
                                                                     for trigger inputs and must be answered
                       {"type":"reset","reason":..}
    plug-in -> mixer   {"out":"<stream>","msg":{...}}                zero or more per trigger
                       {"done":<n>,"health":{...}}                   the trigger `n` is handled (`health` optional)

Two ways to run it, behind the same `Plugin` face:

    sync=True    the mixer waits for `done` (at most `timeout_s`): the outputs are on the bus before the next
                 plug-in runs, exactly as for an in-process plug-in; a replay stays deterministic. The cost is
                 the hop (measured in the lane report) on every trigger.
    sync=False   the mixer does not wait: outputs are put on the bus when they arrive (`pump()`, called by the
                 daemon's event loop when the plug-in's descriptor is readable, and before every send). Consumers
                 read them as "the newest there is". For boxes whose output is not tied to a camera frame.

A plug-in that does not answer in time is reported `fault` and its late answers are dropped; it is restarted
(pipe form) or reconnected (socket form) on the next message after `retry_s`.
"""
import json
import logging
import os
import select
import socket
import subprocess
import time

from .plugin import In, Out, Plugin

log = logging.getLogger("chris_integrator.mixer.remote")
CONTRACT = 1


class RemotePlugin(Plugin):
    remote = True

    def __init__(self, name=None, cmd=None, socket_path=None, cfg=None, sync=True, timeout_s=0.25, start_timeout_s=10.0,
                 retry_s=2.0, env=None, cwd=None, inputs=None, outputs=None):
        self.name = name
        self.cfg = dict(cfg or {})
        self.cmd, self.socket_path, self.sync = cmd, socket_path, sync
        self.timeout_s, self.start_timeout_s, self.retry_s, self.env, self.cwd = timeout_s, start_timeout_s, retry_s, env, cwd
        self.proc = self.sock = None
        self.rfd = self.wfd = None
        self.buf = b""
        self.n = 0
        self.ctx = None
        self.fault = None
        self.down_since = None
        self.remote_health = {}
        self.stats = {"sent": 0, "answers": 0, "timeouts": 0, "restarts": 0, "late": 0, "wait_s": 0.0, "max_wait_ms": 0.0}
        if inputs is not None:
            self.inputs = tuple(In(**i) for i in inputs)
            self.outputs = tuple(Out(**o) for o in (outputs or ()))
        try:
            self._connect(declare=inputs is None)
        except Exception as e:
            if inputs is None:
                raise
            self._down("not started: %s" % e)
        self._triggers = {i.name for i in self.inputs if i.trigger}
        self._reads = {i.name for i in self.inputs}

    # ---- transport -----------------------------------------------------------------------------------
    def _connect(self, declare=False):
        if self.cmd:
            self.proc = subprocess.Popen(self.cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=self.env, cwd=self.cwd)
            self.rfd, self.wfd = self.proc.stdout.fileno(), self.proc.stdin.fileno()
        else:
            self.sock = socket.socket(socket.AF_UNIX)
            self.sock.connect(self.socket_path)
            self.rfd = self.wfd = self.sock.fileno()
        self.buf = b""
        hello = self._read_line(self.start_timeout_s)
        if not hello or hello.get("type") != "plugin":
            self._close()
            raise RuntimeError("remote plug-in %s: no hello line" % (self.name or self.cmd or self.socket_path))
        if hello.get("contract") != CONTRACT:
            self._close()
            raise RuntimeError("remote plug-in speaks contract %r, this mixer %d" % (hello.get("contract"), CONTRACT))
        if declare:
            self.name = self.name or hello.get("name")
            self.inputs = tuple(In(i["name"], bool(i.get("trigger")), bool(i.get("optional")), bool(i.get("feedback")))
                                for i in hello.get("inputs") or ())
            self.outputs = tuple(Out(o["name"], o.get("kind", "measurement"), o.get("time", "frame"),
                                     o.get("public", True), o.get("doc", "")) for o in hello.get("outputs") or ())
        if self.cfg:
            self._write({"type": "config", "cfg": self.cfg})
        self.fault = self.down_since = None

    def _close(self):
        if self.proc is not None:
            try:
                self.proc.stdin.close()
            except OSError:
                pass
            try:
                self.proc.terminate()
                self.proc.wait(2)
            except Exception:
                self.proc.kill()
            self.proc = None
        if self.sock is not None:
            self.sock.close()
            self.sock = None
        self.rfd = self.wfd = None

    def _down(self, why):
        self._close()
        self.fault = why
        self.down_since = time.monotonic()
        log.warning("remote plug-in %s: %s", self.name, why)

    def _write(self, obj):
        data = json.dumps(obj, separators=(",", ":")).encode() + b"\n"
        try:
            while data:
                k = os.write(self.wfd, data)
                data = data[k:]
            return True
        except (OSError, TypeError) as e:
            self._down("write failed: %s" % e)
            return False

    def _read_line(self, timeout):
        """One JSON line, or None when none arrives within `timeout` (0: only what is already there)."""
        end = time.monotonic() + timeout
        while True:
            i = self.buf.find(b"\n")
            if i >= 0:
                line, self.buf = self.buf[:i], self.buf[i + 1:]
                try:
                    return json.loads(line)
                except ValueError:
                    continue
            if self.rfd is None:
                return None
            left = end - time.monotonic()
            r, _, _ = select.select([self.rfd], [], [], max(0.0, left))
            if not r:
                return None
            try:
                chunk = os.read(self.rfd, 1 << 16)
            except OSError:
                chunk = b""
            if not chunk:
                self._down("the plug-in closed its end")
                return None
            self.buf += chunk

    def fileno(self):
        """Descriptor an event loop can watch to call `pump()` (None while the plug-in is down)."""
        return self.rfd

    # ---- the Plugin face -----------------------------------------------------------------------------
    def start(self, ctx):
        self.ctx = ctx

    def _handle(self, obj, ctx, want=None):
        """Apply one line from the plug-in; True when it is the `done` of trigger `want`."""
        if "out" in obj and isinstance(obj.get("msg"), dict):
            ctx.emit(obj["out"], obj["msg"])
            return False
        if "done" in obj:
            if isinstance(obj.get("health"), dict):
                self.remote_health = obj["health"]
            self.stats["answers"] += 1
            if want is not None and obj["done"] != want:
                self.stats["late"] += 1
                return False
            return obj["done"] == want
        return False

    def pump(self):
        """Put on the bus whatever the plug-in has sent (async mode; never blocks)."""
        if self.ctx is None or self.rfd is None:
            return
        while True:
            obj = self._read_line(0.0)
            if obj is None:
                return
            self._handle(obj, self.ctx)

    def on_input(self, stream, msg, ctx):
        if self.rfd is None:
            if self.down_since is not None and time.monotonic() - self.down_since < self.retry_s:
                return
            try:
                self._connect()
                self.stats["restarts"] += 1
            except Exception as e:
                self._down("restart failed: %s" % e)
                return
        trig = stream in self._triggers
        if not self.sync:
            self.pump()
            if self.rfd is None:
                return
        self.n += 1
        if not self._write({"in": stream, "msg": msg, "n": self.n} if trig else {"in": stream, "msg": msg}):
            return
        self.stats["sent"] += 1
        if not (trig and self.sync):
            return
        t0 = time.perf_counter()
        end = t0 + self.timeout_s
        while True:
            obj = self._read_line(max(0.0, end - time.perf_counter()))
            if obj is None:
                if self.rfd is not None:
                    self.stats["timeouts"] += 1
                    self.fault = "no answer within %.0f ms" % (self.timeout_s * 1e3)
                break
            if self._handle(obj, ctx, self.n):
                self.fault = None
                break
        dt = time.perf_counter() - t0
        self.stats["wait_s"] += dt
        self.stats["max_wait_ms"] = max(self.stats["max_wait_ms"], dt * 1e3)

    def reset(self, reason=None):
        if self.rfd is not None:
            self._write({"type": "reset", "reason": reason})

    def health(self):
        h = dict(self.remote_health or {})
        h.setdefault("state", "ok")
        if self.fault:
            h.update(state="fault", reason=self.fault)
        s = self.stats
        h["transport"] = dict(s, mode="sync" if self.sync else "async", pid=self.proc.pid if self.proc else None,
                              wait_ms_mean=round(1e3 * s["wait_s"] / s["sent"], 3) if s["sent"] else None)
        h["transport"].pop("wait_s")
        return h

    def close(self):
        self._close()


def serve(plugin, rfile, wfile, ctx_factory=None):
    """The plug-in side of the wire for a `Plugin` object written for the in-process contract: read the mixer's
    lines from `rfile`, write the answers to `wfile` (binary files). Blocks until the mixer closes its end."""
    def send(obj):
        wfile.write(json.dumps(obj, separators=(",", ":")).encode() + b"\n")
        wfile.flush()

    latest = {}

    class Ctx:
        now = mono_ns = real_ns = 0
        replay = False

        def get(self, name, frame=False):
            return latest.get(name)

        def emit(self, name, msg, line=None):
            send({"out": name, "msg": msg})

        def service(self, name):
            return None

    ctx = Ctx()
    send({"type": "plugin", "contract": CONTRACT, "name": plugin.name, "inputs": [i.as_dict() for i in plugin.inputs],
          "outputs": [o.as_dict() for o in plugin.outputs]})
    plugin.start(ctx)
    for raw in rfile:
        try:
            obj = json.loads(raw)
        except ValueError:
            continue
        if obj.get("type") == "config":
            plugin.configure(obj.get("cfg"))
        elif obj.get("type") == "reset":
            plugin.reset(obj.get("reason"))
        elif "in" in obj:
            msg = obj.get("msg") or {}
            latest[obj["in"]] = msg
            ts = msg.get("ts_mono_ns")
            if isinstance(ts, int):
                ctx.mono_ns, ctx.now = ts, ts / 1e9
            if "n" in obj:
                try:
                    plugin.on_input(obj["in"], msg, ctx)
                    send({"done": obj["n"], "health": plugin.health()})
                except Exception as e:
                    send({"done": obj["n"], "health": {"state": "fault", "reason": "%s: %s" % (type(e).__name__, e)}})
