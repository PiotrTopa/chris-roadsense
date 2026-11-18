#!/usr/bin/env python3
"""Per-frame comparison of two modeld_replay outputs (e.g. --backend ort vs --backend ncnn).

usage: compare_replay.py REF.jsonl TEST.jsonl [--csv OUT.csv]

Both runs keep their own recurrent state (supercombo: features_buffer, prev curvature; vision: the policy's
feature history), so this measures the accumulated difference, not a single-frame one. Works for both road
models (`road_model` supercombo or vision); plan_xy only where both runs have a plan. Metrics per run, then mean/max over the replay:
  lane_y_mean / lane_y_max  |dy| over the 4 lane lines x 33 points (m); lane_y_in_* = the 2 inner lines
  lane_prob                 max |dprob| over the 4 lane lines
  lead_x / lead_v / lead_p  |d| of lead 0 (now hypothesis) at t=0
  plan_xy                   max |d| of plan x and y over 33 points (m)
Acceptance (docs/HARDWARE.md, ncnn backend): mean lane_y_mean < 0.05 m, max lane_y_max < 0.2 m,
lead x within 1 m, lane probs within 0.05.
"""
import json, sys

def load(p):
    return [m for m in map(json.loads, open(p)) if m.get("type") == "road"]

def main():
    a = [x for x in sys.argv[1:] if not x.startswith("--")]
    csv = sys.argv[sys.argv.index("--csv") + 1] if "--csv" in sys.argv else None
    ref, tst = load(a[0]), load(a[1])
    assert len(ref) == len(tst), (len(ref), len(tst))
    rows = []
    for r, t in zip(ref, tst):
        assert r["seq"] == t["seq"]
        dy = [abs(p - q) for lr, lt in zip(r["lane_lines"], t["lane_lines"]) for p, q in zip(lr["y"], lt["y"])]
        dyi = [abs(p - q) for lr, lt in zip(r["lane_lines"][1:3], t["lane_lines"][1:3]) for p, q in zip(lr["y"], lt["y"])]
        lr0, lt0 = r["leads"][0], t["leads"][0]
        rows.append(dict(
            seq=r["seq"],
            lane_y_mean=sum(dy) / len(dy), lane_y_max=max(dy),
            lane_y_in_mean=sum(dyi) / len(dyi), lane_y_in_max=max(dyi),
            lane_prob=max(abs(lr["prob"] - lt["prob"]) for lr, lt in zip(r["lane_lines"], t["lane_lines"])),
            lead_x=abs(lr0["x"][0] - lt0["x"][0]), lead_v=abs(lr0["v"][0] - lt0["v"][0]),
            lead_p=abs(lr0["prob"] - lt0["prob"]), lead_prob_ref=lr0["prob"],
            plan_xy=(max(max(abs(p - q) for p, q in zip(r["plan"][k], t["plan"][k])) for k in ("x", "y"))
                     if "plan" in r and "plan" in t else 0.0),
            model_ms_ref=r["model_ms"], model_ms_test=t["model_ms"]))
    keys = [k for k in rows[0] if k not in ("seq", "lead_prob_ref", "model_ms_ref", "model_ms_test")]
    print(f"{len(rows)} runs  {a[0]} vs {a[1]}")
    print(f"{'metric':16s} {'mean':>9s} {'p95':>9s} {'max':>9s}  (max at seq)")
    for k in keys:
        v = sorted(x[k] for x in rows)
        worst = max(rows, key=lambda x: x[k])
        print(f"{k:16s} {sum(v) / len(v):9.4f} {v[int(0.95 * (len(v) - 1))]:9.4f} {v[-1]:9.4f}  ({worst['seq']})")
    third = len(rows) // 3
    for name, part in (("first third", rows[:third]), ("last third", rows[-third:])):
        print(f"{name:12s} lane_y_mean {sum(x['lane_y_mean'] for x in part) / len(part):.4f}  "
              f"lane_y_max {max(x['lane_y_max'] for x in part):.4f}")
    print(f"model_ms mean ref {sum(x['model_ms_ref'] for x in rows) / len(rows):.1f}, "
          f"test {sum(x['model_ms_test'] for x in rows) / len(rows):.1f}")
    ok = (sum(x["lane_y_mean"] for x in rows) / len(rows) < 0.05 and max(x["lane_y_max"] for x in rows) < 0.2
          and max(x["lead_x"] for x in rows) < 1.0 and max(x["lane_prob"] for x in rows) < 0.05)
    print("ACCEPT" if ok else "REJECT")
    if csv:
        with open(csv, "w") as f:
            f.write(",".join(rows[0]) + "\n")
            for x in rows:
                f.write(",".join(str(x[k]) for k in x) + "\n")
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
