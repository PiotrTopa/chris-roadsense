"""JSON lines over Unix stream sockets (the stack's convention): reconnecting readers for the sources'
sockets, and a line server for consumers in other processes."""
import asyncio
import json
import logging
import os

from ..fastjson import loads_road_trimmed
from ..native import loads

log = logging.getLogger("chris_integrator.io")
LINE_LIMIT = 16 * 1024 * 1024
BACKLOG = 1 << 20


def parse_line(raw):
    """One received line (bytes) -> (message, line as text); (None, None) when it is empty or not a JSON object.
    Raises ValueError on bad JSON. What every source socket's line goes through (tools/cost_bench.py times it)."""
    line = raw.decode("utf-8", "replace").strip()
    if not line:
        return None, None
    msg = loads(line)                      # json.loads, native where built
    return (msg, line) if isinstance(msg, dict) else (None, None)


def parse_line_trimmed(raw):
    """`parse_line` for a stack that runs only the default boxes: a `road` message is parsed without the members
    none of them reads (fastjson.trim_road); the line handed on is the whole one."""
    if not raw.startswith(_ROAD):
        return parse_line(raw)
    line = raw.decode("utf-8", "replace").strip()
    try:
        msg = loads_road_trimmed(raw)
    except ValueError:                     # (bytes that are not UTF-8 land here too)
        msg = loads(line)
    return (msg, line) if isinstance(msg, dict) else (None, None)


_ROAD = b'{"type":"road"'


class Reader:
    """One source socket: connects, reconnects, hands every line to `on_line(msg, line)`."""

    def __init__(self, path, on_line, name=None, backoff_max=3.0, parse=None):
        """parse: bytes -> (message, line), `parse_line` by default."""
        self.path, self.on_line, self.name, self.backoff_max = path, on_line, name or os.path.basename(path), backoff_max
        self.parse = parse or parse_line
        self.connected = False
        self.last = None                   # loop time of the last message
        self.n = 0

    async def run(self):
        backoff = 0.2
        loop = asyncio.get_running_loop()
        while True:
            try:
                reader, writer = await asyncio.open_unix_connection(self.path, limit=LINE_LIMIT)
            except OSError:
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, self.backoff_max)
                continue
            backoff = 0.2
            self.connected = True
            log.info("reading %s", self.path)
            try:
                while True:
                    raw = await reader.readline()
                    if not raw:
                        break
                    try:
                        msg, line = self.parse(raw)
                    except ValueError:
                        log.warning("%s: bad JSON line (%d bytes)", self.name, len(raw))
                        continue
                    if msg is not None:
                        self.last = loop.time()
                        self.n += 1
                        self.on_line(msg, line)
            except (OSError, ValueError, asyncio.LimitOverrunError) as e:
                log.warning("%s: %s", self.path, e)
            finally:
                self.connected = False
                writer.close()
            await asyncio.sleep(backoff)

    def status(self, now):
        return {"connected": self.connected, "last_msg_age_s": round(now - self.last, 3) if self.last else None}


class LineServer:
    """An output channel for other processes: every message of the chosen streams as a JSON line. A client
    gets the last message of each stream when it connects; a first line `{"subscribe": ["lead", ...]}` from the
    client narrows what it gets. A slow client is cut at 1 MiB of backlog."""

    def __init__(self, path, streams=None, greet=None):
        self.path, self.streams, self.greet = path, None if streams is None else tuple(streams), greet
        self.clients = {}                  # writer -> set of streams or None
        self.last = {}                     # stream -> its last line (bytes), or the envelope it is made from
        self.out_bytes = 0
        self.server = None

    def on_envelope(self, env):
        if self.streams is not None and env.stream not in self.streams:
            return
        data = env.line.encode() + b"\n"
        self.last[env.stream] = data
        for w, want in list(self.clients.items()):
            if want is not None and env.stream not in want:
                continue
            if w.transport.get_write_buffer_size() > BACKLOG:
                del self.clients[w]
                w.close()
                continue
            w.write(data)
            self.out_bytes += len(data)

    async def _client(self, reader, writer):
        self.clients[writer] = None
        for data in list(self.last.values()):          # bytes, or an envelope whose line is made now
            writer.write(data if isinstance(data, bytes) else data.line.encode() + b"\n")
        try:
            while True:
                raw = await reader.readline()
                if not raw:
                    break
                try:
                    sub = json.loads(raw).get("subscribe")
                except (ValueError, AttributeError):
                    continue
                if isinstance(sub, list):
                    self.clients[writer] = set(sub)
        except OSError:
            pass
        finally:
            self.clients.pop(writer, None)
            writer.close()

    async def start(self):
        try:
            os.unlink(self.path)
        except OSError:
            pass
        self.server = await asyncio.start_unix_server(self._client, path=self.path, limit=LINE_LIMIT)
        os.chmod(self.path, 0o666)

    def close(self):
        if self.server is not None:
            self.server.close()
