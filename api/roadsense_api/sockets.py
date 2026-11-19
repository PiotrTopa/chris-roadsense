"""One reconnecting asyncio client per daemon socket, newline-delimited JSON."""
import asyncio
import json
import logging
import os

log = logging.getLogger("roadsense_api.sockets")
LINE_LIMIT = 16 * 1024 * 1024


async def daemon_client(state, daemon, path, backoff_min=0.2, backoff_max=5.0, track=True):
    """`track` False: a second socket of a daemon (mapd's ego.sock); it does not own the daemon's `connected` flag."""
    backoff = backoff_min
    while True:
        try:
            reader, writer = await asyncio.open_unix_connection(path, limit=LINE_LIMIT)
        except OSError:
            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, backoff_max)
            continue
        log.info("connected to %s", path)
        if track:
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
            if track:
                state.connected[daemon] = False
            writer.close()
        log.info("disconnected from %s", path)
        await asyncio.sleep(backoff)


def start_clients(state, rundir, daemons):
    tasks = [asyncio.create_task(daemon_client(state, d, os.path.join(rundir, d + ".sock")),
                                 name="sock-" + d) for d in daemons]
    if "mapd" in daemons:       # the car's own motion as mapd accepted it (`ego` messages), for the lead tracker
        tasks.append(asyncio.create_task(daemon_client(state, "mapd", os.path.join(rundir, "ego.sock"), track=False,
                                                       backoff_max=2.0), name="sock-ego"))
        # the car as the host describes it (`vehicle`) and the scans of its range sensors (`objects`)
        tasks.append(asyncio.create_task(daemon_client(state, "mapd", os.path.join(rundir, "vehicle.sock"), track=False,
                                                       backoff_max=2.0), name="sock-vehicle"))
    # optional: a separate lane source for the display (`display_lanes` messages, view.py); absent on most hosts
    tasks.append(asyncio.create_task(daemon_client(state, "laned", os.path.join(rundir, "laned.sock"), track=False),
                                     name="sock-laned"))
    return tasks
