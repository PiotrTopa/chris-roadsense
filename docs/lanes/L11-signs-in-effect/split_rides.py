#!/usr/bin/env python3
"""Split the recorder's loop.jsonl into one file per ride (camera messages carry the ride). usage: split_rides.py loop.jsonl outdir slug..."""
import json, sys
src, out, slugs = sys.argv[1], sys.argv[2], set(sys.argv[3:])
files, cur = {}, None
with open(src) as f:
    for line in f:
        try:
            m = json.loads(line)["m"]
        except (ValueError, KeyError):
            continue
        if m.get("type") == "camera":
            cur = m.get("ride")
        if cur in slugs:
            if cur not in files:
                files[cur] = open(f"{out}/{cur}.jsonl", "w")
            files[cur].write(line)
for k, fh in files.items():
    fh.close(); print(k)
