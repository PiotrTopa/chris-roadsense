#!/usr/bin/env python3
"""Tables from chain/*.json (chain_table.py): usage: bisect_table.py RIDE CONFIG..."""
import json, sys, os
D = os.path.expanduser("~/datasets/l32/chain"); ride = sys.argv[1]; cfgs = sys.argv[2:]
J = {c: json.load(open(f"{D}/{ride}.{c}.json")) for c in cfgs if os.path.exists(f"{D}/{ride}.{c}.json")}
def g(c, stage, b, k="off"):
    v = J[c].get(stage, {}).get(b)
    return "-" if not v or v.get(k) is None else f"{v[k]:+.2f}"
def n(c, stage, b):
    v = J[c].get(stage, {}).get(b); return 0 if not v else v["n"]
print("**What the MFD draws: lead minus the drawn road centre at the lead's distance, m (+ = lead to the right); in brackets the direction of the drawn centre 5-20 m, deg**\n")
cells = [("6 lead 15-30 m", "0-0.3", "standing, lead 15-30 m"), ("6 lead 30-60 m", "0-0.3", "standing, lead 30-60 m"), ("6 lead < 15 m", "3-7", "3-7 m/s, lead < 15 m"),
         ("6 lead 15-30 m", "7-11", "7-11 m/s, lead 15-30 m"), ("6 lead 30-60 m", "11-15", "11-15 m/s, lead 30-60 m"), ("6 lead 60+ m", "11-15", "11-15 m/s, lead 60+ m"), ("6 lead 60+ m", "15-99", "15+ m/s, lead 60+ m")]
print("| configuration | " + " | ".join(c[2] for c in cells) + " |\n|---|" + "---|" * len(cells))
for c in J:
    print(f"| {c} | " + " | ".join(f"{g(c, s, b)} ({g(c, s, b, 'centre_dir')})" if g(c, s, b) != "-" else "-" for s, b, _ in cells) + " |")
print("\n**The same offset stage by stage, own speed 11-15 m/s (frames with the lead and both inner lines), m**\n")
stages = ["1 raw road", "2 topics lead/lanes", "3 view", "4a scene (tracked part)", "4b scene vw (view part)", "5a mfd director, both lines", "5b mfd, renderer's centre rule"]
print("| configuration | " + " | ".join(s[2:].strip() for s in stages) + " |\n|---|" + "---|" * len(stages))
for c in J:
    print(f"| {c} | " + " | ".join(g(c, s, "11-15") for s in stages) + " |")
print("\n**Stage by stage, by own speed (offset m / frames used)**\n")
bins = ["0-0.3", "0.3-3", "3-7", "7-11", "11-15", "15-99"]
for c in J:
    print(f"\n{c}\n\n| stage | " + " | ".join(b + " m/s" for b in bins) + " |\n|---|" + "---|" * len(bins))
    for s in stages:
        if s in J[c]: print(f"| {s[2:].strip()} | " + " | ".join(f"{g(c, s, b)} / {n(c, s, b)}" for b in bins) + " |")
