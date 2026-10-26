#!/usr/bin/env python3
"""Temporal-smoothing check on synthetic 1 fps sequences built from per-image zero-shot scores.

Sequence: K runs of `run` seconds, each run drawn from one class (cycling through the classes the
dataset has >= `run` images of), one random image of that class per second; with `--noise p`, each
second is replaced with probability p by an image of another random class (a wrong frame).
The smoother is a Python replica of envd/smoother.h (window mean + hysteresis + min hold); the
C++ one is checked against it with envd_replay on one written-out sequence (--write-list).

    stability_sim.py --emb bdd_X.npz --classes classes_X.json --group light --label-col timeofday --dataset bdd
                     [--run 30] [--runs 12] [--noise 0.2] [--window 10] [--hysteresis 0.1] [--min-hold 3] [--seed 0]
                     [--write-list seq.txt]
Reports: raw per-frame arg-max accuracy and flips, smoothed label accuracy, switch latency (s) per
true change, extra flips (label changes that are not true changes).
"""
import argparse
import collections
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from zeroshot_eval import MAP, scores  # noqa: E402


class Smoother:
    def __init__(self, n, window, hyst, hold):
        self.n, self.window, self.hyst, self.hold = n, window, hyst, hold
        self.hist = collections.deque()
        self.label, self.changed_t = -1, 0.0

    def update(self, p, t):
        self.hist.append((t, p))
        while self.hist and t - self.hist[0][0] > self.window:
            self.hist.popleft()
        s = np.mean([q for _, q in self.hist], axis=0)
        best = int(s.argmax())
        if self.label < 0:
            self.label, self.changed_t = best, t
        elif best != self.label and s[best] > s[self.label] + self.hyst and t - self.changed_t >= self.hold:
            self.label, self.changed_t = best, t
        return self.label


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--emb", required=True)
    ap.add_argument("--classes", required=True)
    ap.add_argument("--group", required=True)
    ap.add_argument("--label-col", required=True)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--run", type=int, default=30)
    ap.add_argument("--runs", type=int, default=12)
    ap.add_argument("--noise", type=float, default=0.0)
    ap.add_argument("--window", type=float, default=10)
    ap.add_argument("--hysteresis", type=float, default=0.1)
    ap.add_argument("--min-hold", type=float, default=3)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--write-list", default="")
    ap.add_argument("--root", default="")
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    d = np.load(a.emb)
    doc = json.load(open(a.classes))
    classes, probs = scores(d["emb"], doc, a.group)
    m = MAP[a.dataset][a.label_col]
    raw = d[a.label_col]
    by_class = collections.defaultdict(list)
    for i, r in enumerate(raw):
        if r in m:
            by_class[classes.index(m[r])].append(i)
    usable = [c for c in sorted(by_class) if len(by_class[c]) >= a.run]
    seq_true, seq_idx = [], []
    prev = -1
    for k in range(a.runs):
        c = usable[k % len(usable)]
        if c == prev and len(usable) > 1:
            c = usable[(k + 1) % len(usable)]
        prev = c
        for _ in range(a.run):
            src = c
            if a.noise > 0 and rng.random() < a.noise:
                src = int(rng.choice([u for u in usable if u != c]))
            seq_true.append(c)
            seq_idx.append(int(rng.choice(by_class[src])))
    sm = Smoother(len(classes), a.window, a.hysteresis, a.min_hold)
    raw_lab, sm_lab = [], []
    for t, i in enumerate(seq_idx):
        raw_lab.append(int(probs[i].argmax()))
        sm_lab.append(sm.update(probs[i], float(t)))
    seq_true, raw_lab, sm_lab = np.array(seq_true), np.array(raw_lab), np.array(sm_lab)
    true_changes = [t for t in range(1, len(seq_true)) if seq_true[t] != seq_true[t - 1]]
    sm_changes = [t for t in range(1, len(sm_lab)) if sm_lab[t] != sm_lab[t - 1]]
    raw_flips = sum(1 for t in range(1, len(raw_lab)) if raw_lab[t] != raw_lab[t - 1])
    lat, used = [], set()
    bounds = true_changes + [len(seq_true)]
    for k, tc in enumerate(true_changes):
        after = [t for t in sm_changes if tc <= t < bounds[k + 1] and sm_lab[t] == seq_true[tc] and t not in used]
        lat.append(after[0] - tc if after else None)
        if after:
            used.add(after[0])
    extra = len(sm_changes) - len(used)  # label changes that are not the detection of a true change
    print(f"{a.group}: {len(seq_true)} s, {len(true_changes)} true changes, noise {a.noise:.0%}, window {a.window} s, "
          f"hysteresis {a.hysteresis}, hold {a.min_hold} s")
    print(f"  raw arg-max: accuracy {np.mean(raw_lab == seq_true) * 100:.1f} %, {raw_flips} flips")
    print(f"  smoothed:    accuracy {np.mean(sm_lab == seq_true) * 100:.1f} %, {len(sm_changes)} label changes, "
          f"latency s: {[x for x in lat]} (median {np.median([x for x in lat if x is not None]) if any(x is not None for x in lat) else '-'}), "
          f"missed {sum(1 for x in lat if x is None)}, extra flips {extra}")
    if a.write_list:
        with open(a.write_list, "w") as f:
            for i in seq_idx:
                f.write(os.path.join(a.root, str(d["files"][i])) + "\n")
        with open(a.write_list + ".labels", "w") as f:
            for t in range(len(seq_true)):
                f.write(f"{t},{classes[seq_true[t]]},{classes[sm_lab[t]]}\n")


if __name__ == "__main__":
    main()
