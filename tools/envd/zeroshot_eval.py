#!/usr/bin/env python3
"""Score cached image embeddings (embed_dataset.py) against the class text embeddings
(text_embeddings.py) and print per-class recall, macro/micro accuracy and confusion rows.

    zeroshot_eval.py --emb bdd_vitb32_openai.npz --classes envd-classes-vitb32_openai.json \
                     --group weather --label-col weather --dataset bdd [--split A|B] [--md]

The label mapping of the lane pre-registration (docs/lanes/L2-envd.md) is fixed here; dataset
labels that map to nothing are dropped, those listed in REPORT_ONLY are reported as a prediction
split but excluded from the headline.
"""
import argparse
import hashlib
import json
import sys

import numpy as np

MAP = {
    "bdd": {
        "scene": {"highway": "highway", "city street": "city_street", "residential": "residential",
                  "parking lot": "parking_lot", "tunnel": "tunnel", "gas stations": "gas_station"},
        "weather": {"clear": "clear", "overcast": "overcast", "rainy": "rain", "snowy": "snow", "foggy": "fog"},
        "timeofday": {"daytime": "day", "dawn/dusk": "dusk_dawn", "night": "night"},
    },
    "dawn": {"weather": {"foggy": "fog", "rainy": "rain", "snowy": "snow"}},
    "places": {"scene": {c: c for c in ("highway", "city_street", "residential", "forest_road", "rural_road",
                                        "parking_lot", "underground_parking", "gas_station", "bridge")}},
}
REPORT_ONLY = {"bdd": {"weather": ["partly cloudy"]}}
MIN_MACRO = 20


def split_of(name):
    return "A" if int(hashlib.sha1(name.encode()).hexdigest(), 16) % 2 == 0 else "B"


def scores(emb, classes_doc, group):
    g = classes_doc["groups"][group]
    t = np.array(g["emb"], dtype=np.float32)
    logits = classes_doc["logit_scale"] * emb @ t.T
    logits -= logits.max(1, keepdims=True)
    p = np.exp(logits)
    p /= p.sum(1, keepdims=True)
    return g["classes"], p


def evaluate(emb, labels, classes, probs, md=False, title=""):
    pred = probs.argmax(1)
    present = sorted(set(labels), key=lambda c: classes.index(c))
    rows, macro = [], []
    for c in present:
        idx = np.array([i for i, l in enumerate(labels) if l == c])
        n = len(idx)
        rec = float((pred[idx] == classes.index(c)).mean())
        conf = {}
        for j in pred[idx]:
            conf[classes[j]] = conf.get(classes[j], 0) + 1
        top = sorted(conf.items(), key=lambda kv: -kv[1])[:4]
        rows.append((c, n, rec, ", ".join(f"{k} {v / n * 100:.0f}%" for k, v in top)))
        if n >= MIN_MACRO:
            macro.append(rec)
    idx = np.array([i for i, l in enumerate(labels)])
    micro = float((pred[idx] == np.array([classes.index(l) for l in labels])).mean())
    out = []
    if md:
        out.append(f"| class | n | recall | predicted as |")
        out.append("|---|---|---|---|")
        for c, n, rec, top in rows:
            out.append(f"| {c} | {n} | {rec * 100:.1f} | {top} |")
        out.append(f"| **macro (n>={MIN_MACRO})** | {len(labels)} | **{np.mean(macro) * 100:.1f}** | micro {micro * 100:.1f} |")
    else:
        out.append(f"{title}")
        for c, n, rec, top in rows:
            out.append(f"  {c:20s} n={n:5d} recall={rec * 100:5.1f}  pred: {top}")
        out.append(f"  macro={np.mean(macro) * 100:.1f} (classes with n>={MIN_MACRO}) micro={micro * 100:.1f}")
    return "\n".join(out), float(np.mean(macro)), micro


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--emb", required=True)
    ap.add_argument("--classes", required=True)
    ap.add_argument("--group", required=True, choices=("place", "weather", "light"))
    ap.add_argument("--label-col", required=True)
    ap.add_argument("--dataset", required=True, choices=tuple(MAP))
    ap.add_argument("--split", default="", choices=("", "A", "B"))
    ap.add_argument("--md", action="store_true")
    a = ap.parse_args()
    d = np.load(a.emb)
    doc = json.load(open(a.classes))
    classes, probs = scores(d["emb"], doc, a.group)
    raw = d[a.label_col]
    files = d["files"]
    keep = np.ones(len(raw), bool)
    if a.split:
        keep &= np.array([split_of(f) == a.split for f in files])
    m = MAP[a.dataset][a.label_col]
    sel = np.array([r in m for r in raw]) & keep
    labels = [m[r] for r in raw[sel]]
    text, macro, micro = evaluate(d["emb"][sel], labels, classes, probs[sel], a.md,
                                  f"{a.emb} {a.group} ({a.dataset}/{a.label_col}{' split ' + a.split if a.split else ''})")
    print(text)
    for lab in REPORT_ONLY.get(a.dataset, {}).get(a.label_col, []):
        idx = np.where((raw == lab) & keep)[0]
        if len(idx) == 0:
            continue
        pred = probs[idx].argmax(1)
        cnt = {}
        for j in pred:
            cnt[classes[j]] = cnt.get(classes[j], 0) + 1
        top = ", ".join(f"{k} {v / len(idx) * 100:.0f}%" for k, v in sorted(cnt.items(), key=lambda kv: -kv[1]))
        print(f"| ({lab}, report only) | {len(idx)} | - | {top} |" if a.md else f"  ({lab}: n={len(idx)} pred: {top})")


if __name__ == "__main__":
    main()
