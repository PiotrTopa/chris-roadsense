#!/usr/bin/env python3
"""Writes road_raw_vision.jsonl: three synthetic `road` lines with the layout of a vision road model's output
(chris-roadsense modeld with the vision + policy model: every member, 33-point plan and lines, 6-point leads,
numbers at 5 significant digits as modeld prints them). The values are a made-up drive at about 21 m/s on a
gently curving road; no recording is involved. tests/test_road_trim.py reads the file.

    python3 tests/data/make_road_raw.py > tests/data/road_raw_vision.jsonl
"""
import json
import math
import random

T = [10.0 * (i / 32) ** 2 for i in range(33)]
X = [192.0 * (i / 32) ** 2 for i in range(33)]
LT = [0, 2, 4, 6, 8, 10]


def g(v):
    """5 significant digits, integers without a point (as modeld writes them)."""
    v = float(f"{v:.5g}")
    return int(v) if v == int(v) and abs(v) < 1e15 else v


def road(k, rng):
    v = 21.25 + 0.3 * k
    curv = 0.0003 * math.sin(0.7 * k + 0.4)
    n = lambda s: rng.gauss(0.0, s)          # noqa: E731
    px = [v * t + 0.1 * t * t for t in T]
    plan = {"t": [g(t) for t in T], "x": [g(x) for x in px], "y": [g(0.5 * curv * x * x + n(1e-4)) for x in px],
            "z": [g(0.004 * x + n(1e-4)) for x in px], "vx": [g(v + 0.2 * t + n(0.01)) for t in T],
            "vy": [g(n(5e-4)) for _ in T], "vz": [g(0.05 + n(0.002)) for _ in T],
            "ax": [g(0.2 - 0.01 * t + n(0.005)) for t in T], "ay": [g(curv * v * v + n(0.005)) for _ in T],
            "az": [g(0.09 + n(0.01)) for _ in T], "roll": [g(n(1e-4)) for _ in T],
            "pitch": [g(-1e-4 * t) for t in T], "yaw": [g(curv * x) for x in px],
            "roll_rate": [g(0.005 + n(5e-4)) for _ in T], "pitch_rate": [g(-0.003 + n(5e-4)) for _ in T],
            "yaw_rate": [g(curv * v + n(2e-4)) for _ in T], "x_std": [g(0.001 + 0.08 * t * t) for t in T],
            "y_std": [g(0.0007 + 0.02 * t) for t in T], "z_std": [g(0.0007 + 0.01 * t) for t in T],
            "prob": None, "held": True, "age_ms": 100}
    lines = []
    for off, prob in ((-5.6, 0.90797), (-1.85, 0.99282), (1.85, 0.98991), (5.6, 0.0032225)):
        lines.append({"x": [g(x) for x in X], "y": [g(off + 0.5 * curv * x * x + n(0.003)) for x in X],
                      "z": [g(1.21 + 0.004 * x + n(0.002)) for x in X],
                      "y_std": [g(0.2 + 0.004 * x + n(0.002)) for x in X], "prob": g(prob + n(0.001))})
    edges = [{"x": [g(x) for x in X], "y": [g(off + 0.5 * curv * x * x + n(0.005)) for x in X],
              "z": [g(1.19 + 0.004 * x + n(0.002)) for x in X], "y_std": [g(0.3 + 0.006 * x) for x in X]}
             for off in (-4.2, 3.9)]
    leads = []
    for j, prob in enumerate((0.047958, 0.046987, 0.064653)):
        x0 = 120.0 + 10 * j
        leads.append({"prob": g(prob), "t_offset": 0, "t": LT, "x": [g(x0 + 20 * t + n(0.5)) for t in LT],
                      "y": [g(n(0.5)) for _ in LT], "v": [g(20.4 - 0.05 * t) for t in LT],
                      "a": [g(0.03 + n(0.01)) for _ in LT], "x_std": [g(12 + 0.5 * t) for t in LT],
                      "y_std": [g(0.5 + 0.1 * t) for t in LT], "v_std": [g(2.3 - 0.03 * t) for t in LT],
                      "a_std": [g(0.27 - 0.005 * t) for t in LT]})
    pose = {"vx": g(v), "vx_std": None, "vy": g(n(0.02)), "vz": g(0.08 + n(0.005)), "roll_rate": g(n(5e-4)),
            "pitch_rate": g(-0.0016 + n(3e-4)), "yaw_rate": g(curv * v), "vy_std": g(0.03), "vz_std": g(0.04),
            "roll_rate_std": g(0.0038), "pitch_rate_std": g(0.0034), "yaw_rate_std": g(0.0011)}
    pose = {key: pose[key] for key in ("vx", "vy", "vz", "roll_rate", "pitch_rate", "yaw_rate", "vx_std", "vy_std",
                                       "vz_std", "roll_rate_std", "pitch_rate_std", "yaw_rate_std")}
    pose["vx_std"] = g(0.76 + n(0.01))
    p = lambda base, m: [g(base * (1.0 + 1.5 * i) * m) for i in range(5)]   # noqa: E731
    meta = {"engaged_prob": g(0.45 + n(0.01)),
            "disengage_probs": {"gas": p(0.001, 1), "brake": p(0.0055, 1), "steer_override": p(0.094, 1),
                                "brake_3ms2": p(7.7e-05, 1), "brake_4ms2": p(2.2e-05, 1), "brake_5ms2": p(7.8e-06, 1),
                                "gas_press": [g(0.987 + 0.0002 * i) for i in range(5)]},
            "blinkers": {"left": [g(0.0012 * (i + 1) ** 1.5) for i in range(6)],
                         "right": [g(0.0011 * (i + 1) ** 1.5) for i in range(6)]},
            "hard_brake_pred": p(7.8e-06, 1), "fcw": False}
    desire = [g(0.99987)] + [g(1.6e-07 + n(1e-8)) for _ in range(7)]
    ts = 1_000_000_000_000 + k * 2_650_000_000
    return {"type": "road", "seq": 1000 + 53 * k, "ts_mono_ns": ts, "ts_real_ns": 1_700_000_000_000_000_000 + ts,
            "model_ms": g(45.0 + n(3)), "prep_ms": g(6.0 + n(1)), "frame_seq_prev": 996 + 53 * k, "pair_dt_ms": 200,
            "frames_skipped": 0, "v_ego": g(v), "effective_hz": 20, "rate_limited": False, "rate_hz": g(15.1 + n(0.1)),
            "governor_level": 0, "limited": False, "steer_delay": 0.4, "valid": True, "road_model": "vision",
            "road_model_fallback": False, "policy_hz": 5, "policy_ms": g(0.006 if k < 2 else 9.3),
            "plan": plan, "lane_lines": lines, "road_edges": edges, "leads": leads, "pose": pose,
            "road_transform": [0, 0, g(1.337), g(0.0126), g(-0.0029), g(0.00098)],
            "road_transform_std": [g(0.00099), g(0.00098), g(0.047), g(0.0021), g(0.0019), g(0.0012)],
            "desired_curvature": None, "desired_curvature_std": None, "desire_state": desire,
            "desire_pred": [[g(0.99992)] + [g(7.6e-08 * (j + 1)) for j in range(7)] for _ in range(4)], "meta": meta}


def main():
    rng = random.Random(51)
    for k in range(3):
        print(json.dumps(road(k, rng), separators=(",", ":")))


if __name__ == "__main__":
    main()
