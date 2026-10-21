"""One reconnecting asyncio client per daemon socket, newline-delimited JSON."""
import asyncio
import json
import logging
import os

log = logging.getLogger("pocopilot_api.sockets")
LINE_LIMIT = 16 * 1024 * 1024


async def daemon_client(state, daemon, path, backoff_min=0.2, backoff_max=5.0):
    backoff = backoff_min
    while True:
        try:
            reader, writer = await asyncio.open_unix_connection(path, limit=LINE_LIMIT)
        except OSError:
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, backoff_max)
            continue
        log.info("connected to %s", path)
        state.connected[daemon] = True
        backoff = backoff_min
        try:
            while True:
                raw = await reader.readline()
                if not raw:
                    break
                line = raw.decode("utf-8", "replace").strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                except ValueError:
                    log.warning("%s: bad JSON line (%d bytes)", daemon, len(line))
                    continue
                if isinstance(msg, dict):
                    state.on_message(daemon, msg, line)
        except (OSError, ValueError, asyncio.LimitOverrunError) as e:
            log.warning("%s: %s", daemon, e)
        finally:
            state.connected[daemon] = False
            writer.close()
        log.info("disconnected from %s", path)
        await asyncio.sleep(backoff)


def start_clients(state, rundir, daemons):
    return [asyncio.create_task(daemon_client(state, d, os.path.join(rundir, d + ".sock")),
                                name="sock-" + d) for d in daemons]
