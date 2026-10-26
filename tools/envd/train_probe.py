#!/usr/bin/env python3
"""Linear probe on frozen image embeddings: multinomial logistic regression per group, trained on
split A of a labelled embedding file and scored on split B (same metric as zeroshot_eval.py).
Optionally writes the probe as class "text" embeddings into a copy of the class file, so envd can
use it unchanged (the probe's weight rows replace the prompt embeddings; its bias is folded in by
appending a constant to the embedding is NOT possible in envd, so the probe is trained without bias:
scores = logit_scale * emb . w_c, with w_c free). With --init-zeroshot the L2 penalty pulls each w_c
towards its prompt embedding (classes the data lacks keep their zero-shot behaviour).

    train_probe.py --emb bdd_X.npz --classes classes_X.json --group weather --label-col weather --dataset bdd
                   [--extra dawn_X.npz:weather] [--l2 1e-3] [--out classes_X_probe.json]

Only numpy + torch (CPU). Probe weights trained on BDD/DAWN (research licences) are not committed.
"""
import argparse
import json
import sys
import os

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from zeroshot_eval import MAP, evaluate, scores, split_of  # noqa: E402


def load(path, dataset, label_col, group_classes):
    d = np.load(path)
    m = MAP[dataset][label_col]
    raw = d[label_col]
    sel = np.array([r in m for r in raw])
    y = np.array([group_classes.index(m[r]) for r in raw[sel]])
    files = d["files"][sel]
    return d["emb"][sel], y, files


def train(X, y, n_classes, l2, scale, init=None, steps=300):
    """init: the zero-shot class embeddings; the L2 penalty then pulls W towards them instead of
    towards zero, so classes without training data keep their prompt embedding."""
    Xt = torch.from_numpy(X).float()
    yt = torch.from_numpy(y).long()
    W0 = torch.zeros(n_classes, X.shape[1]) if init is None else torch.from_numpy(init).float()
    W = W0.clone()
    W.requires_grad_(True)
    opt = torch.optim.LBFGS([W], lr=1, max_iter=steps, line_search_fn="strong_wolfe")
    # class-balanced loss: the taxonomy is what matters, not BDD's class frequencies
    cnt = torch.bincount(yt, minlength=n_classes).float().clamp(min=1)
    wts = (cnt.sum() / cnt / n_classes).clamp(max=20.0)

    def closure():
        opt.zero_grad()
        logits = scale * Xt @ W.T
        loss = torch.nn.functional.cross_entropy(logits, yt, weight=wts) + l2 * ((W - W0) ** 2).sum()
        loss.backward()
        return loss

    opt.step(closure)
    return W.detach().numpy()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--emb", required=True)
    ap.add_argument("--classes", required=True)
    ap.add_argument("--group", required=True)
    ap.add_argument("--label-col", required=True)
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--extra", action="append", default=[], help="more labelled data: path.npz:dataset:label_col (split A/B by file name like the main set; A trains, B is reported)")
    ap.add_argument("--l2", type=float, default=1e-3)
    ap.add_argument("--init-zeroshot", action="store_true", help="start from the prompt embeddings (keeps classes without data usable)")
    ap.add_argument("--out", default="")
    ap.add_argument("--md", action="store_true")
    a = ap.parse_args()
    doc = json.load(open(a.classes))
    g = doc["groups"][a.group]
    classes = g["classes"]
    X, y, files = load(a.emb, a.dataset, a.label_col, classes)
    isA = np.array([split_of(f) == "A" for f in files])
    XA, yA, XB, yB = X[isA], y[isA], X[~isA], y[~isA]
    extras_B = []
    for ex in a.extra:
        p, ds, col = ex.split(":")
        Xe, ye, fe = load(p, ds, col, classes)
        eA = np.array([split_of(f) == "A" for f in fe])  # extra sets are split the same way: A trains, B tests
        XA = np.concatenate([XA, Xe[eA]])
        yA = np.concatenate([yA, ye[eA]])
        extras_B.append((p, Xe[~eA], ye[~eA]))
    init = np.array(g["emb"], np.float32) if a.init_zeroshot else None
    W = train(XA, yA, len(classes), a.l2, doc["logit_scale"], init=init)
    labelsB = [classes[i] for i in yB]
    # zero-shot on B
    _, pz = scores(XB, doc, a.group)
    tz, mz, uz = evaluate(XB, labelsB, classes, pz, a.md, "zero-shot on B")
    # probe on B
    logits = doc["logit_scale"] * XB @ W.T
    logits -= logits.max(1, keepdims=True)
    pp = np.exp(logits)
    pp /= pp.sum(1, keepdims=True)
    tp, mp, up = evaluate(XB, labelsB, classes, pp, a.md, "probe on B")
    print(f"train A: {len(yA)} images, test B: {len(yB)} images, l2 {a.l2}")
    print("zero-shot on B:" if a.md else "")
    print(tz)
    print("probe on B:" if a.md else "")
    print(tp)
    print(f"macro zero-shot {mz * 100:.1f} -> probe {mp * 100:.1f}; micro {uz * 100:.1f} -> {up * 100:.1f}")
    for p, Xe, ye in extras_B:
        le = [classes[i] for i in ye]
        _, pz = scores(Xe, doc, a.group)
        _, mz, _ = evaluate(Xe, le, classes, pz, False, "")
        lg = doc["logit_scale"] * Xe @ W.T
        lg -= lg.max(1, keepdims=True)
        pe = np.exp(lg)
        pe /= pe.sum(1, keepdims=True)
        te, me, _ = evaluate(Xe, le, classes, pe, a.md, f"probe on {p} split B")
        print(f"{p} split B ({len(ye)} images): macro zero-shot {mz * 100:.1f} -> probe {me * 100:.1f}")
        print(te)
    if a.out:
        doc["groups"][a.group]["emb"] = [[round(float(v), 6) for v in row] for row in W]
        doc["groups"][a.group]["probe"] = {"trained_on": [a.emb] + a.extra, "l2": a.l2, "split": "A"}
        json.dump(doc, open(a.out, "w"), separators=(",", ":"))
        print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
