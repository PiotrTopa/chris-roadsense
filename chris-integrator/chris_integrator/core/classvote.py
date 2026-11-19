"""What kind of road user an object is: a vote over the object's life (non-critical metadata, lane L42).

A class detector says per frame what it sees, with a score, and it flickers: the same car is a `car` in one
frame and a `truck` in the next, a cyclist is a `person` until the bike is in view. What a consumer draws must
not flicker with it. So the label of an object is a vote over its whole life:

* every observation adds its score (times a quality weight: a box cut by the detector's region counts less) to
  its label's tally; tallies fade with a long half-life (`HALF_LIFE_S`), never per frame, and their sum is
  held at `CAP` (all scaled down together), so the vote is a share over the recent life of the object and a
  track handed to another vehicle relearns in seconds, not never;
* the first label is shown once `MIN_VOTES` observations are in and it leads the runner-up by `FIRST_RATIO`;
  it is sticky: another label replaces it only when its tally is `SWITCH_RATIO` times the shown one's;
* `confidence` = the shown label's share of all tallies, times the mean score the detector gave that label,
  times how much evidence there is (1 - exp(-observations / `EVIDENCE_SCALE`)): a label the detector only ever
  gives with a low score (on the car's camera: vans called `truck` at 0.3-0.6) stays a low-confidence label
  however often it is repeated, and two observations do not make a confident class;
* below `MIN_VOTES` observations or `MIN_CONFIDENCE` the class is unknown (None): "unknown" beats a confident
  wrong answer, and a consumer draws its generic shape then.

Labels are open vocabulary, lower case; common aliases are folded (`person` -> `human`, `bicycle` -> `bike`,
`motor` / `motorcycle` -> `motorbike`, `rider` -> `bike`: a rider without the vehicle under it was seen, most
often on a bicycle). Nothing here is read by gating, association, existence or the lead selection.
"""
import math

ALIASES = {"person": "human", "pedestrian": "human", "bicycle": "bike", "cyclist": "bike", "rider": "bike",
           "motorcycle": "motorbike", "motor": "motorbike", "lorry": "truck", "coach": "bus", "train": "tram"}
HALF_LIFE_S = 30.0
CAP = 8.0
SWITCH_RATIO = 2.0
FIRST_RATIO = 1.5
MIN_VOTES = 2
MIN_CONFIDENCE = 0.35
EVIDENCE_SCALE = 1.0


def norm_label(label):
    if not isinstance(label, str) or not label.strip():
        return None
    s = label.strip().lower()
    return ALIASES.get(s, s)


def _conf(v):
    if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v):
        return min(1.0, max(0.0, float(v)))
    return 0.5


def vote(mem, label, conf, src, t, weight=1.0):
    """Fold one observation into a vote kept in the dict `mem` (created on first use). Returns the label."""
    label = norm_label(label)
    if label is None:
        return None
    w = _conf(conf) * max(0.0, float(weight))
    if not mem:
        mem.update({"w": {}, "c": {}, "src": {}, "t": t, "n": 0, "shown": None})
    dt = t - mem["t"] if t is not None and mem["t"] is not None else 0.0
    if dt > 0.0:
        k = 0.5 ** (dt / HALF_LIFE_S)
        for key in mem["w"]:
            mem["w"][key] *= k
            mem["c"][key] *= k
    q = max(0.0, float(weight))
    mem["w"][label] = mem["w"].get(label, 0.0) + w            # score-weighted tally (the vote)
    mem["c"][label] = mem["c"].get(label, 0.0) + q            # observation tally (w / c = the mean score)
    tot = sum(mem["w"].values())
    if tot > CAP:
        for key in mem["w"]:
            mem["w"][key] *= CAP / tot
            mem["c"][key] *= CAP / tot
    mem["src"][label] = src
    mem["t"] = t if t is not None and (mem["t"] is None or t > mem["t"]) else mem["t"]
    mem["n"] += 1
    best = max(mem["w"], key=mem["w"].get)
    shown = mem.get("shown")
    if shown is None:
        second = max((v for k, v in mem["w"].items() if k != best), default=0.0)
        if mem["n"] >= MIN_VOTES and mem["w"][best] >= FIRST_RATIO * second:
            mem["shown"] = best
    elif best != shown and mem["w"][best] >= SWITCH_RATIO * mem["w"].get(shown, 0.0) and mem["n"] >= MIN_VOTES:
        mem["shown"] = best
    return label


def result(mem, T=None):
    """{"label", "confidence", "source", "age_s", "votes"} of a vote, or None while unknown."""
    if not mem or not mem.get("w") or mem.get("shown") is None:
        return None
    label = mem["shown"]
    total = sum(mem["w"].values())
    if total <= 0.0:
        return None
    wl, cl = mem["w"].get(label, 0.0), mem["c"].get(label, 0.0)
    mean_score = wl / cl if cl > 0.0 else 0.0
    conf = wl / total * mean_score * (1.0 - math.exp(-sum(mem["c"].values()) / EVIDENCE_SCALE))
    if conf < MIN_CONFIDENCE:
        return None
    age = 0.0 if T is None or mem.get("t") is None else max(0.0, T - mem["t"])
    return {"label": label, "confidence": round(min(1.0, conf), 2), "source": mem["src"].get(label),
            "age_s": round(age, 2), "votes": mem["n"]}
