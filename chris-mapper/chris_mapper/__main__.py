"""The map box as a process.

    python3 -m chris_mapper --plugin [--pack PACK]        a plug-in of a chris-integrator mixer, out of process:
                                                          JSON lines on stdin / stdout (the plug-in wire
                                                          contract: a hello line with the declared inputs and
                                                          outputs, then `{"in": "ego_state", "msg": ..., "n": k}`
                                                          in, `{"out": "map", "msg": ...}` and `{"done": k}` out)
    python3 -m chris_mapper --replay FILE [--pack PACK]   `ego_state` lines of a file in, `map` lines out

This file needs nothing of chris-integrator: the contract is the lines.
"""
import argparse
import json
import sys

from .box import INPUTS, NAME, OUTPUTS, MapBox


def plugin(box, rfile, wfile):
    def send(obj):
        wfile.write(json.dumps(obj, separators=(",", ":")).encode() + b"\n")
        wfile.flush()
    send({"type": "plugin", "contract": 1, "name": NAME, "inputs": INPUTS, "outputs": OUTPUTS})
    for raw in rfile:
        try:
            obj = json.loads(raw)
        except ValueError:
            continue
        if obj.get("type") == "config":
            cfg = obj.get("cfg") or {}
            if cfg.get("pack") and cfg["pack"] != box.pack_path:
                box.pack_path, box._pack_id, box._pack_t = cfg["pack"], None, None
            box.cfg.update({k: v for k, v in cfg.items() if k in box.cfg})
        elif obj.get("type") == "reset":
            box.reset(obj.get("reason"))
        elif obj.get("in") == "ego_state":
            try:
                out = box.on_ego_state(obj.get("msg") or {})
                if out is not None:
                    send({"out": "map", "msg": out})
                health = box.health()
            except Exception as e:
                health = {"state": "fault", "reason": "%s: %s" % (type(e).__name__, e)}
            if "n" in obj:
                send({"done": obj["n"], "health": health})


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plugin", action="store_true")
    ap.add_argument("--replay")
    ap.add_argument("--pack")
    a = ap.parse_args(argv)
    box = MapBox({"pack": a.pack} if a.pack else None)
    if a.replay:
        with open(a.replay) as f:
            for ln in f:
                es = json.loads(ln)
                if es.get("type") == "ego_state":
                    out = box.on_ego_state(es)
                    if out is not None:
                        sys.stdout.write(json.dumps(out, separators=(",", ":")) + "\n")
        return
    plugin(box, sys.stdin.buffer, sys.stdout.buffer)


if __name__ == "__main__":
    main()
