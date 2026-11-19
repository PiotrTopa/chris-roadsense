"""Track-list scoring against the simulator's truth, shared by the sensor tests."""
import numpy as np


def per_vehicle(run, pred, t0=20.0, gate=(3.5, 1.6)):
    """Over the frames: for every true vehicle that `pred` selects, is there a confirmed track on it, how many
    (duplicates), which ids over time, and the position error of the nearest one."""
    w = run.w
    hit = n = dup = 0
    ids, err = {}, []
    for t, tr in run.tracks:
        if t < t0:
            continue
        conf = [k for k in tr["tracks"] if k["state"] != "tentative" and "ghost" not in k["flags"]]
        for o in w.targets_at(t + w.t0):
            if not pred(o):
                continue
            n += 1
            near = [k for k in conf if abs(k["x_m"] - o["x"]) < gate[0] and abs(k["y_m"] - o["y"]) < gate[1]]
            if near:
                hit += 1
                dup += len(near) > 1
                k = min(near, key=lambda k: abs(k["x_m"] - o["x"]))
                ids.setdefault(o["id"], []).append(k["id"])
                err.append((k["x_m"] - o["x"], k["y_m"] - o["y"]))
    switches = sum(sum(1 for a, b in zip(v, v[1:]) if a != b) for v in ids.values())
    e = np.array(err) if err else np.zeros((1, 2))
    return {"n": n, "tracked": hit / max(n, 1), "dup": dup / max(hit, 1), "switches": switches,
            "rms_x": float(np.sqrt(np.mean(e[:, 0] ** 2))), "rms_y": float(np.sqrt(np.mean(e[:, 1] ** 2)))}
