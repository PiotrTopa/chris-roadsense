"""Receiver fixes (`gnss`, mapd's relay of the feeder's messages).

The reference car's head unit (docs/DESIGN.md sections 1 and 3a): about one fix a second, stamped at arrival,
positions about 2 s old and consistent in scale with the odometer; a claimed `hacc` that is too optimistic;
a speed field that is a slower, smoothed output (1.5-9 s behind, low) and is therefore not fused: it is a
separate witness source. Nothing is fused while the car stands (a standing receiver wanders).

The error model (L47). A fix is `position(t - delay) + slowly varying error + white noise`:

* The white part is small: from one fix to the next the receiver moves as the odometer says to 0.75-1.55 m
  (robust sigma, four drives). The error that matters is the slowly varying one: raw fixes sit 7-10 m (median)
  beside the road they are on, the same way for 5-20 s. Before L47 that error was fused as white noise of
  `hacc x 1..8` with a 2.5 m correlated state: twenty fixes then average to a quarter of it, and the stated
  position uncertainty was 4-5 m where held-out fixes lay 10-19 m away. It is now two states (east, north),
  first-order Gauss-Markov over 20 s, in units of the receiver's present error level `sigma_b`: they enter the
  rows multiplied by `sigma_b`, so the level may change from fix to fix without touching the filter's process.
* `sigma_b = level x hacc x scale`, plus what the receiver's own motion says (below). `level` is how much
  larger the slowly varying error is than the claimed `hacc`: unknown for an unknown receiver (prior 2), and
  measured while a map match is used, raw against raw: the distance of the fixes from the matched road's line
  on straights (`MapMatchSource`), robustly, over the last minute of such fixes. `scale` (1-2.5) is the second
  moment of those states as the filter finds them, in units of the level: above 1 the level is too small.
* The receiver's own motion against the car's, raw against raw: the displacement from one fix to the next
  against the distance the car's speed sensors measured between their times of validity, and against the
  direction of the receiver's own earlier track turned by the yaw the car made since (the estimator's heading
  and position are not asked: they may be the wrong party; nothing is concluded while the car's own speed
  sources disagree). A receiver with few satellites keeps moving for ten seconds after the car has stopped
  and crawls while it accelerates: what it moves and the car does not is held back (subtracted from its
  fixes, decaying over 45 s, with `sigma_b` raised by as much), and the delay state is not taught by such
  fixes. A single step beyond what the pair explains is a jump of the receiver's error: held back the same
  way, and the position's own covariance takes the step in (the receiver may have returned to the truth), so
  the position stays on dead reckoning and the map across a multipath excursion and returns to the receiver
  gradually. A receiver whose held-back steps add up to more than `hold_m` (mirrored coordinates, a position
  somewhere else) is not used at all until its fixes come back to the estimate.
* Headings: at an anchoring the heading is the direction of the receiver's own track (pieces driven forwards,
  none longer than the car drove, two in a row that agree) carried to now by the car's yaw; the course field
  only when the yaw sensors were alive since the time it describes and the car has not turned since.
* The delay: a state with a prior per receiver (the reference head unit: 2.0 +- 0.6 s). At constant speed the
  delay and the along-track position are one quantity, and with a map match in the loop a filter left to
  itself settles the difference wherever its other states are stiffest. The delay is therefore *measured*,
  raw against raw, by something that involves neither the position nor the map: the lag at which the
  receiver's step lengths fit the distances the car drove (`_delay_raw`; over the last minute of fixes, when
  the speed varied enough for the lag to show). That is a row on the delay state with its own sigma. On the
  reference car the lag found this way is 1.75-2.5 s on three drives and 7.25 s on the fourth, where the
  receiver had 3-5 satellites and reported a smoothed track: the delay is a property of the receiver's
  present way of tracking, not a constant of the device. With 5 satellites or fewer the delay's uncertainty
  is opened to 1.5 s until that measurement narrows it. Position rows still carry the delay's uncertainty,
  and teach it only while the receiver moves as the car does.
"""
import math

import numpy as np

from ..native import round                  # builtins.round, native where built
from ..core import ego as E
from ..core.ego import AuxState
from ..core.registry import NS, Meas, Source, fin

FIXES = ("2d", "3d", "dgps", "rtk")


def _repeat(msg, mem):
    key = (msg.get("lat"), msg.get("lon"), msg.get("speed_mps"), msg.get("course_deg"))
    rep = key == mem.get("key")
    mem["key"] = key
    mem["n"] = mem.get("n", 0) + 1
    mem["rep"] = mem.get("rep", 0) + (1 if rep else 0)
    return rep


class GnssSource(Source):
    id = "gnss"
    group = "gnss"
    rank = 0
    types = ("gnss",)
    stale_s = 5.0
    info = "receiver positions (delay estimated; slowly varying error as states; own motion checked against the car's)"
    aux = (AuxState("tau", 2.0, 0.6, 5e-4, lo=0.0, hi=9.0, unit="s", doc="a fix is this old at its stamp"),
           AuxState("ue", 0.0, 1.0, tau_s=20.0, unit="x sigma_b", doc="the receiver's slowly varying position error, east, "
                    "in units of its present level (`bias_sigma_m`)"),
           AuxState("un", 0.0, 1.0, tau_s=20.0, unit="x sigma_b", doc="... north"))
    straight_s = 8.0             # the course is a slow output too: used only this long after the last turn
    min_sigma_m = 3.0            # a claimed hacc is not taken below this
    white_m = 1.5                # fix-to-fix noise (per axis)
    level0 = 2.0                 # slowly varying error / claimed hacc, before anything has measured it
    level_lo, level_hi = 0.5, 6.0
    level_n, level_min_n = 60, 12        # ... measured against the matched road over this many fixes (at least)
    scale_max = 2.5
    scale_n = 30.0               # fixes the noise scale averages over
    bias_max_m = 250.0
    course_sigma = math.radians(15.0)
    alt_sigma_m = 10.0
    tau_max = 9.0                # the delay state's upper bound (the filter keeps 10 s of history)
    # the delay measured from the receiver's step lengths against the car's distances
    lag_step, lag_max = 0.5, 10.0        # lags tried, s
    lag_n, lag_min_n, lag_every = 60, 15, 5      # steps kept, steps needed, a measurement every so many steps
    lag_depth, lag_depth_s = 0.75, 3.0   # the fit at the lag found is this much better than 3 s either side of it
    lag_huber_m = 4.0
    lag_max_sigma = 2.0          # a lag known worse than this is not a measurement
    lag_floor_sigma = 0.4
    lag_change_s = 1.5           # measured lag this far (and 3 sigma) from the state, three times running: the receiver tracks
                                 # differently now, and the state is put there
    # the receiver's displacement against the car's, fix to fix
    step_gap_s = 3.5             # ... over at most this long
    step_n, step_min_n = 9, 6    # fixes the receiver's unexplained motion is judged over
    wander_s = 5.0               # an excess of r m/s means an error that may have grown by r x this unnoticed
    jump_m, jump_k = 8.0, 0.5    # a step beyond 8 m + half the car's own displacement is a jump of the receiver's error
    jump_decay_s = 45.0
    unsteady_mps = 2.5           # two steps in three beyond this: the receiver is not moving as the car does
    hold_m = 250.0               # held-back steps beyond this: the receiver is not used
    release_n = 3                # ... until this many fixes in a row lie within the estimate's reach again
    rejoin_n = 10                # ... or, once dead reckoning has run out, it has moved as the car does this many fixes in a row
    anchor_wait_n = 4            # fixes an anchoring waits for a heading from the receiver's own track before the course field
    course_age_s = 2.5           # the course field describes the track up to this long before the fix's time of validity
    leg_m = 8.0                  # at low speed the track's direction is taken over a leg of this length
    dir_min_n = 2                # the track's direction counts as a heading once this many pieces in a row agreed
    dir_len_k, dir_len_m = 0.35, 2.0     # ... none longer than the car drove meanwhile by more than this share and this many metres
    few_sats, few_sats_n = 5, 9  # at most this many satellites (median of the last fixes): the receiver leans on its own
    few_sats_tau_sigma = 1.5     # filter, and how late its positions are is not known to better than this (s) until measured
    flip_rad, flip_n = 1.0, 3    # the heading this far off the direction the receiver moves in, this many times running:
                                 # not something position rows can mend; the heading is placed anew
    # How old a fix is at its stamp is a property of the receiver and of how its host stamps it. The prior of the
    # delay state is the reference receiver's (a head unit: 2.0 +- 0.6 s, measured); a receiver that names itself
    # (`source` in the message) may have its own: {tag: (delay_s, sigma_s)}, extended from the configuration
    # (`integrator.json`: {"gnss_delay": {"mytag": [0.2, 0.3]}}). `synthetic` is the vehicle input API's tag of
    # a simulated drive: its fixes are stamped at their time of validity.
    delays = {"synthetic": (0.0, 0.05)}

    def _receiver(self, msg, ctx):
        """Another receiver took over (the tag changed): its delay is not the old one's, and its positions are not
        a continuation of the old ones (a simulated drive starts somewhere else). The delay state goes to that
        receiver's prior and the position is anchored anew on its fixes."""
        mem = ctx.mem
        tag = msg.get("source")
        if "receiver_tag" in mem and tag == mem["receiver_tag"]:
            return
        first = "receiver_tag" not in mem
        if not first:
            ctx.integ.new_position_source("receiver changed to %s" % (tag or "an unnamed one"))
        mem = ctx.mem                                # (forgetting the position clears this source's memory)
        mem["receiver_tag"] = tag
        table = dict(self.delays, **{k: tuple(v) for k, v in (ctx.cfg.get("gnss_delay") or {}).items()})
        a = self.aux[0]
        d0, s0 = table.get(tag, (a.x0, a.sigma0))
        if tag in table:
            ctx.integ.reset_aux(self.ix["tau"], d0, s0)
        elif not first:
            ctx.integ.reset_aux(self.ix["tau"], a.x0, a.sigma0)
        ctx.integ.model.set_clamp(self.ix["tau"], 0.0, self.tau_max)

    # ---- the receiver's own motion against the car's (raw against raw) -------------------------------------------
    def _motion(self, ctx, mem, ts, e, n, tau):
        """Compare the displacement since the previous fix with what the car covered between the two times of
        validity: its length against the odometer, its direction against the direction of the receiver's own
        earlier track turned by the yaw the car made since (the estimator's heading is not used: it may be the
        wrong party). Updates mem: `wander` (m/s of unexplained motion, 0 while steady), `jump` (east,
        north: held-back steps, decaying), `dir` (time, direction of the receiver's track), `ok_run`."""
        prev = mem.get("prev")
        mem["prev"] = (ts, e, n)
        J = mem.get("jump")
        if prev is None:
            return None
        dt = ts - prev[0]
        if J is not None:
            k = math.exp(-max(dt, 0.0) / self.jump_decay_s)
            J = mem["jump"] = (J[0] * k, J[1] * k)
            if math.hypot(*J) < 0.5:
                J = mem["jump"] = None
        if not 0.2 < dt <= self.step_gap_s:
            mem["pend"] = []
            return None
        integ = ctx.integ
        t_mid = prev[0] - tau + 0.5 * dt
        dist = integ.dist_between(prev[0] - tau, ts - tau)
        dpsi = integ.yaw_between(prev[0] - tau, ts - tau)
        if dist is None or dpsi is None or self._speed_in_doubt(integ):
            # the car's own motion is not known over this stretch (a gap), or its speed sources disagree with
            # each other: then the receiver is not the one to blame for moving differently
            mem["pend"] = []
            return None
        chord = dist * (math.sin(0.5 * dpsi) / (0.5 * dpsi) if abs(dpsi) > 1e-3 else 1.0)
        de, dn = e - prev[1], n - prev[2]
        d = math.hypot(de, dn)
        ref = mem.get("dir")
        turn = integ.yaw_between(ref[0], t_mid) if ref is not None and t_mid - ref[0] < 12.0 else None
        if turn is not None and abs(chord) >= 3.0:       # the whole vector, in the frame of the receiver's own track
            pred = ref[1] + turn
            me, mn = de - chord * math.cos(pred), dn - chord * math.sin(pred)
        else:                                            # ... else the lengths only
            pred = None
            k = (d - abs(chord)) / d if d > 1e-6 else 0.0
            me, mn = k * de, k * dn
        mis = math.hypot(me, mn)
        mem["steps"] = mem.get("steps", 0) + 1
        jump = mis > self.jump_m + self.jump_k * abs(chord)
        was = mem.get("wander", 0.0) > 0.0
        if jump:
            mem["jumps"] = mem.get("jumps", 0) + 1
            mem["last_jump_m"] = round(mis, 1)
            mem["ok_run"] = 0
            mem["leg"] = None
            # a step of the receiver, alone: it left the truth, or it returned to it. In the second case the
            # position, which has been following it, is wrong by that step: its uncertainty takes the step in
            # (not while the receiver is unsteady anyway: then dead reckoning and the map carry the position)
            if not was and integ.anchored and not getattr(integ, "_reanchor_pending", False):
                integ.widen_position(me, mn)
        else:
            mem["ok_run"] = mem.get("ok_run", 0) + 1
            self._delay_raw(ctx, mem, prev[0], ts, d)
            # The direction of the receiver's track: of one step when it is long enough, else of a leg of several
            # steps driven forwards without much turning (walking pace in a car park gives 1-3 m a step). Never
            # of a step driven backwards: the track then points away from the heading (a three-point turn anchored
            # the heading 180 degrees wrong, and the position ran 700 m the other way before it was noticed).
            # ... and never of a piece longer than the car drove meanwhile: a receiver that has just been standing
            # (and wandering) settles for a few seconds when the car drives off, and that motion is not the car's
            # (two such pieces anchored the heading 180 degrees wrong at a start). A piece shorter than the car
            # drove (a receiver that crawls behind) still points the car's way.
            obs = tm = None
            if dist < -0.3:
                mem["leg"] = None
            elif d >= 5.0 and chord >= 3.0:
                if d - chord <= self.dir_len_k * chord + self.dir_len_m:
                    obs, tm = math.atan2(dn, de), t_mid
                mem["leg"] = None
            elif dist > 0.3:
                leg = mem.get("leg")
                turned = integ.yaw_between(leg[0], ts - tau) if leg is not None else None
                if leg is None or turned is None or abs(turned) > 0.35 or ts - tau - leg[0] > 15.0:
                    mem["leg"] = (prev[0] - tau, prev[1], prev[2])
                elif math.hypot(e - leg[1], n - leg[2]) >= self.leg_m:
                    driven = integ.dist_between(leg[0], ts - tau)
                    if driven is not None and math.hypot(e - leg[1], n - leg[2]) - driven <= self.dir_len_k * driven + self.dir_len_m:
                        obs, tm = math.atan2(n - leg[2], e - leg[1]), 0.5 * (leg[0] + ts - tau)
                    mem["leg"] = None
            if obs is not None:
                p_ = None
                if ref is not None and tm - ref[0] < 12.0:
                    y_ = integ.yaw_between(ref[0], tm)
                    p_ = None if y_ is None else ref[1] + y_
                # (a slow follower of the observed directions, carried by the car's yaw between them; `dir_ok`
                # counts the observations in a row that agreed with the one before, carried: one step of a
                # receiver that has just come out of a gap may point anywhere)
                agree = p_ is not None and abs((obs - p_ + math.pi) % (2.0 * math.pi) - math.pi) <= 0.35
                mem["dir_ok"] = mem.get("dir_ok", 0) + 1 if agree else 1
                mem["dir"] = (tm, obs if p_ is None or abs((obs - p_ + math.pi) % (2.0 * math.pi) - math.pi) > 0.6
                              else p_ + 0.5 * ((obs - p_ + math.pi) % (2.0 * math.pi) - math.pi))
                mem["dir_n"] = mem.get("dir_n", 0) + 1
            # how fast the receiver moves differently from the car: the lengths only (the direction of its own
            # track is a noisy reference), what two in three of the last steps exceed (single steps are jumps, above).
            # Healthy: 0.5-1.0 m/s on the reference car's drives; with 3-4 satellites 4-7 m/s for 10-20 s at a time
            mq = mem.setdefault("mq", [])
            mq.append(abs(d - abs(chord)) / dt)
            del mq[:-self.step_n]
            low = sorted(mq)[len(mq) // 3] if len(mq) >= self.step_min_n else 0.0      # two steps in three are above this
            mem["wander"] = low if low > self.unsteady_mps else 0.0
        # What the receiver moved and the car did not is held back from its fixes: a jump at once, and every step
        # while the receiver is not moving as the car does (with the last steps before that was noticed). A healthy
        # receiver's steps are not accumulated: they are noise, and a scale difference of a few per cent against the
        # odometer would add up to tens of metres.
        pend = mem.setdefault("pend", [])
        now_ = mem.get("wander", 0.0) > 0.0
        if jump or now_:
            add = [(me, mn)] + (pend if now_ and not was else [])
            del pend[:]
            J = mem["jump"] = ((J[0] if J else 0.0) + sum(a_[0] for a_ in add), (J[1] if J else 0.0) + sum(a_[1] for a_ in add))
        else:
            pend.append((me, mn))
            del pend[:-2]
        return mis

    @staticmethod
    def _speed_in_doubt(integ):
        """Do two fused sources of the car's speed disagree at present (whatever is done about it)?"""
        if integ.disputed.get("speed") is not None:
            return True
        for (hi, lo, q), p in integ.pairs.items():
            if q == "speed" and p.verdict == "disagree" and p.c.get("mode") == "share" \
                    and not integ.groups[hi].witness and not integ.groups[lo].witness:
                return True
        return False

    def _delay_raw(self, ctx, mem, t_a, t_b, d):
        """The receiver's delay from its own motion: the lag L at which its step lengths fit the distances the car
        drove between `stamp - L` of the two fixes, over the last `lag_n` steps. No position, no heading, no map
        enters. Leaves `mem["lag"] = (L, sigma)` when the steps determine it (the speed varied), for `measure`."""
        q = mem.setdefault("lagq", [])
        q.append((t_a, t_b, d))
        del q[:-self.lag_n]
        mem["lag_k"] = mem.get("lag_k", 0) + 1
        if mem["lag_k"] % self.lag_every or len(q) < self.lag_min_n:
            return
        integ = ctx.integ
        nl = int(round(self.lag_max / self.lag_step)) + 1
        rows = []
        for a, b, dr in q:
            ds = [integ.dist_between(a - k * self.lag_step, b - k * self.lag_step) for k in range(nl)]
            if all(x is not None for x in ds):           # (the car's motion is known that far back from this step)
                rows.append((dr, [abs(x) for x in ds]))
        if len(rows) < self.lag_min_n:
            return
        h = self.lag_huber_m
        cost = []
        for k in range(nl):
            c = 0.0
            for dr, ds in rows:
                r = abs(dr - ds[k])
                c += 0.5 * r * r if r <= h else h * (r - 0.5 * h)
            cost.append(c)
        k = min(range(nl), key=cost.__getitem__)
        if k == 0 or k == nl - 1 or k * self.lag_step > self.tau_max:
            return                                       # at the edge of what was tried: not a minimum
        w = int(round(self.lag_depth_s / self.lag_step))
        if min(cost[max(k - w, 0)], cost[min(k + w, nl - 1)]) * self.lag_depth < cost[k]:
            return                                       # a shallow minimum (the speed hardly varied): says nothing
        den = cost[k - 1] - 2.0 * cost[k] + cost[k + 1]
        L = self.lag_step * (k + (0.5 * (cost[k - 1] - cost[k + 1]) / den if den > 1e-9 else 0.0))
        # how sharply the steps determine it: residual variance over the summed squared sensitivity
        sens = sum(((ds[k + 1] - ds[k - 1]) / (2.0 * self.lag_step)) ** 2 for _, ds in rows)
        res = sum(min(abs(dr - ds[k]), 2.0 * h) ** 2 for dr, ds in rows) / len(rows)
        if sens <= 1e-6:
            return
        sig = math.sqrt(max(res, 1.0) / sens)
        mem["lag_raw"] = (round(L, 2), round(sig, 2), len(rows))
        if sig > self.lag_max_sigma:
            return
        # consecutive measurements share most of their steps: each is worth its share of new ones
        mem["lag"] = (L, max(sig * math.sqrt(len(rows) / self.lag_every), self.lag_floor_sigma), sig)

    def _heading_now(self, ctx, mem, ts):
        """The car's heading now from the direction of the receiver's track, turned by the yaw the car made since
        that piece of track was driven (None: no such piece since the last gap of the yaw sensors)."""
        ref = mem.get("dir")
        integ = ctx.integ
        if ref is None or ts - ref[0] > 15.0 or integ.now is None or mem.get("dir_ok", 0) < self.dir_min_n:
            return None
        turn = integ.yaw_between(ref[0], integ.now)
        return None if turn is None else ref[1] + turn

    def level(self, mem):
        """Slowly varying error / claimed hacc: measured against the matched road when there is a map, else the prior."""
        return min(max(mem.get("level", self.level0), self.level_lo), self.level_hi)

    def offer_cross(self, ctx, dist_m, hacc):
        """`MapMatchSource`: a raw fix lies `dist_m` beside the line of a confidently matched road, on a straight.
        The robust scale of those distances, less the car's own place on the road, is this receiver's slowly
        varying error; against the claimed `hacc` it is the receiver's level."""
        mem = ctx.mem
        q = mem.setdefault("cross", [])
        q.append((abs(dist_m), max(float(hacc), self.min_sigma_m) if fin(hacc) else 10.0))
        del q[:-self.level_n]
        if len(q) >= self.level_min_n:
            d = sorted(x[0] for x in q)
            h = sorted(x[1] for x in q)
            sig = math.sqrt(max((1.4826 * d[len(d) // 2]) ** 2 - 2.0 ** 2, 0.25))
            mem["level"] = sig / h[len(h) // 2]
            mem["cross_sigma_m"] = round(sig, 1)

    def measure(self, msg, ctx):
        self._receiver(msg, ctx)
        mem = ctx.mem
        rep = _repeat(msg, mem)
        mem["fix"], mem["hacc"], mem["receiver"] = msg.get("fix"), msg.get("hacc_m"), msg.get("source")
        lat, lon = msg.get("lat"), msg.get("lon")
        if msg.get("fix") not in FIXES or not fin(lat) or not fin(lon):
            return []
        ts = msg["ts_mono_ns"] / NS
        integ = ctx.integ
        itau = self.ix["tau"]
        x = ctx.x
        tau = float(x[itau])
        if integ.origin is None:
            integ.origin = (float(lat), float(lon))
        e, n = E.to_local(integ.origin, float(lat), float(lon))
        sats = msg.get("sats")
        if fin(sats) and sats > 0 and not rep:
            # Few satellites: the receiver reports more of its own filter and less of the sky (on the reference
            # car, the two drives with 3-5 satellites are the ones whose fixes crawl and catch up, 7 s behind by
            # their own motion on one of them; with 6-10 the lag is 1.75-2.5 s). When the count falls that low the
            # delay state's uncertainty is opened to say so; the measurement by motion then narrows it again.
            q = mem.setdefault("sats", [])
            q.append(sats)
            del q[:-self.few_sats_n]
            few = len(q) >= 3 and sorted(q)[len(q) // 2] <= self.few_sats
            if few and not mem.get("few"):
                if math.sqrt(max(float(ctx.P[itau, itau]), 0.0)) < self.few_sats_tau_sigma:
                    integ.reset_aux(itau, tau, self.few_sats_tau_sigma)
                    mem["few_opened"] = mem.get("few_opened", 0) + 1
            mem["few"] = few
        if not rep:
            self._motion(ctx, mem, ts, e, n, tau)
        sp = ctx.speed_known()
        if sp is None or abs(sp[0]) < 0.3:
            mem["skipped_standing"] = mem.get("skipped_standing", 0) + (0 if rep else 1)
            if sp is not None and sp[1] < 0.3 and self._at_rest(mem, ts, e, n) and self._unplaced(ctx):
                # the car stands, the receiver has come to rest, and there is no usable position: it is placed on
                # this fix (heading unknown until the car moves). Before L47 a position lost just before a stop,
                # or a cold start in a car park, stayed unknown until the car drove at 3 m/s.
                hacc = msg.get("hacc_m")
                hacc = max(float(hacc), self.min_sigma_m) if fin(hacc) else 10.0
                sb = min(self.level(mem) * hacc, self.bias_max_m)
                mem["sigma_b"] = sb
                mem["jump"] = None
                integ.abs_position(self.id, ts, float(lat), float(lon), self.white_m, itau, bias=(self.ix["ue"], self.ix["un"]),
                                   bias_coef=sb, own_heading=True, place=(0.0, False), sigma_total=math.hypot(self.white_m, sb))
                mem["placed_standing"] = mem.get("placed_standing", 0) + 1
            return [] if rep else self._alive(ts)
        mem.pop("rest", None)
        if rep:
            return []
        self._heading_check(ctx, mem, ts, sp)
        hacc = msg.get("hacc_m")
        hacc = max(float(hacc), self.min_sigma_m) if fin(hacc) else 10.0
        # The slowly varying error as the filter finds it, in units of its stated level: its second moment is 1
        # when the level is right. Where it runs above 1 (the level was measured across the road, on straights;
        # along the road the receiver's lag adds to it) the level is raised by that much, within bounds.
        iu, in_ = self.ix["ue"], self.ix["un"]
        u2 = 0.5 * (float(x[iu]) ** 2 + float(x[in_]) ** 2 + float(ctx.P[iu, iu]) + float(ctx.P[in_, in_]))
        k_ = 1.0 / min(mem.get("u2_k", 0) + 1, self.scale_n)
        mem["u2_k"] = mem.get("u2_k", 0) + 1
        mem["u2"] = mem.get("u2", 1.0) + k_ * (min(u2, 16.0) * mem.get("scale_u2", 1.0) - mem.get("u2", 1.0))
        mem["scale_u2"] = min(max(mem["u2"], 1.0), self.scale_max ** 2)
        scale = math.sqrt(min(max(mem.get("scale2", 1.0), mem["scale_u2"], 1.0), self.scale_max ** 2))
        J = mem.get("jump")
        jm = math.hypot(*J) if J else 0.0
        # the level, what is held back (it is only as good as the comparison that produced it), and what an
        # unsteady receiver may have gathered before that was noticed
        wander = mem.get("wander", 0.0) * self.wander_s
        sigma_b = min(math.sqrt((self.level(mem) * hacc * scale) ** 2 + wander ** 2 + jm ** 2), self.bias_max_m)
        mem["sigma_b"] = sigma_b
        white = self.white_m

        def adapt(nu, s0, t):
            # the innovation of this fix against what the model promises (innovation-based adaptation)
            r = nu * nu / (s0 + white * white)
            k = 1.0 / min(mem.get("scale_k", 0) + 1, self.scale_n)
            mem["scale_k"] = mem.get("scale_k", 0) + 1
            mem["scale2"] = mem.get("scale2", 1.0) + k * (min(r, 25.0) - mem.get("scale2", 1.0))
        if self._held(ctx, mem, ts, jm):
            return self._alive(ts)
        course, gs = msg.get("course_deg"), msg.get("speed_mps")
        c = ctx.cfg
        if abs(float(x[E.W])) > 0.05 or sp[0] < 5.0:
            mem["turn_t"] = ts
        straight = ts - mem.get("turn_t", ts) >= self.straight_s
        course_ok = fin(course) and sp[0] >= c["anchor_min_speed"] and fin(gs) and gs >= 2.0
        # a receiver that is not moving as the car moves says nothing about its own delay
        unsteady = wander + jm > 5.0
        mem["unsteady"] = unsteady
        heading = None
        mem["course_prev"], mem["course_now"] = mem.get("course_now"), course
        if not integ.anchored or getattr(integ, "_reanchor_pending", False):
            # placing the heading: from the receiver's own track carried to now by the car's yaw; the course field
            # (the track's direction of `delay` s ago, wrong by the turn made since) only after a few fixes
            # without such a piece of track
            heading = self._heading_now(ctx, mem, ts)
            mem["anchor_wait"] = mem.get("anchor_wait", 0) + 1
            # (the course field is as good as the track while it stands still from one fix to the next and the
            # car drives straight: nothing was turned since)
            # The course field is the track's direction of some seconds ago (the delay, and the receiver's own
            # smoothing): it is the heading now only if the car has not turned since, and that is known only if
            # the yaw sensors were alive since. (A car that turned inside a gap of all input came out of it with
            # a receiver still reporting the old course, steadily: anchored on it, the position drove 300 m down
            # the road it had left.) So: never a course from before or inside a gap; a steady one at once, any
            # other after a few fixes without a piece of the receiver's own track.
            lc = mem.get("course_prev")
            since = getattr(integ, "known_since", None)
            covered = since is not None and since <= ts - tau - self.course_age_s
            turned = integ.yaw_between(ts - tau - self.course_age_s, integ.now) if covered and integ.now is not None else None
            fresh = turned is not None and abs(turned) < 0.1
            steady = fin(course) and fin(lc) and abs((course - lc + 180.0) % 360.0 - 180.0) < 8.0
            # (no position at all yet, or it was forgotten on request: nothing says the car turned in a gap; a
            # steady course of a car that drives straight now is taken)
            first = not integ.anchored and abs(float(x[E.W])) < 0.03 and ctx.P[E.W, E.W] < 0.01
            if heading is None and not ((fresh and (steady or mem["anchor_wait"] > self.anchor_wait_n)) or (first and steady)):
                course_ok = False
        else:
            mem["anchor_wait"] = 0
        place = None
        extra = abs(sp[0]) * (tau + 1.0)
        if heading is None and not course_ok and getattr(integ, "_reanchor_pending", False) and integ.anchored \
                and self._unplaced_m(ctx) > 1.3 * math.sqrt(2.0 * (sigma_b ** 2 + extra ** 2)) + 5.0:
            # no heading yet (a gap has just ended, or the car has just driven off) and the position is known
            # clearly worse than this fix gives it: the position goes on the fix, give or take what the car drove
            # since the fix's time in a direction that is not known; the heading follows with a later fix
            place = (extra, True)
        out = integ.abs_position(self.id, ts, float(lat), float(lon), white, itau,
                                 float(course) if fin(course) else None, course_ok, cb=adapt, heading=heading, own_heading=True,
                                 place=place,
                                 bias=(self.ix["ue"], self.ix["un"]), bias_coef=sigma_b, offset=J,
                                 consider=(itau,) if unsteady else None,
                                 sigma_total=math.hypot(white, sigma_b))
        lag = mem.pop("lag", None)
        if lag is not None:
            L, sig, sharp = lag
            i_t = np.array([itau], dtype=np.intp)
            one_ = np.array([1.0])
            st = math.sqrt(max(float(ctx.P[itau, itau]), 0.0))
            far = abs(L - tau) > max(self.lag_change_s, 3.0 * math.hypot(sharp, st))
            mem["lag_far"] = mem.get("lag_far", 0) + 1 if far and sharp <= 1.0 else 0
            if mem["lag_far"] >= 3:
                # the receiver tracks differently now (fewer satellites: a smoothed track): the delay is what its
                # motion says, to what that measurement is worth
                mem["lag_far"] = 0
                mem["lag_resets"] = mem.get("lag_resets", 0) + 1
                integ.reset_aux(itau, min(max(L, 0.0), self.tau_max), max(sharp, self.lag_floor_sigma))
                integ.fdi.event("gnss", "delay", None, "the receiver's delay is %.1f s by its own motion against the car's "
                                "(the state said %.1f s): taken from there" % (L, tau), quantity="position")
            else:
                out.append(Meas("delay", ts, L, sig, lambda x: (x[itau], i_t, one_), "gnss.delay", robust=True, local=True))
        if place is not None:
            mem["placed_moving"] = mem.get("placed_moving", 0) + 1
        if not integ.anchored:
            return out or self._alive(ts)
        if course_ok and straight and not unsteady:
            i_h = np.array([E.PSI, itau], dtype=np.intp)

            def h_c(x, tau0=tau):
                return x[E.PSI] - x[E.W] * (x[itau] - tau0), i_h, np.array([1.0, -x[E.W]])
            # the innovation is wrapped: compare against the heading unwrapped to the state's branch
            zc = E.course_to_psi(float(course))
            zc = float(x[E.PSI]) + ((zc - float(x[E.PSI]) + math.pi) % (2.0 * math.pi) - math.pi)
            out.append(Meas("heading", ts - tau, zc, self.course_sigma, h_c, "gnss.course", robust=True,
                            local=not c["gnss_absolute"]))
        alt = msg.get("alt_m")
        if fin(alt):
            i_z = np.array([E.Z], dtype=np.intp)
            one = np.array([1.0])
            out.append(Meas("alt", ts - tau, float(alt), self.alt_sigma_m, lambda x: (x[E.Z], i_z, one),
                            "gnss.alt", robust=True))
        return out or self._alive(ts)

    def _unplaced(self, ctx):
        integ = ctx.integ
        if not integ.anchored or getattr(integ, "_reanchor_pending", False):
            return True
        P = ctx.P
        return math.sqrt(max(float(P[E.PE, E.PE] + P[E.PN, E.PN]), 0.0)) > integ.cfg["position_max_hacc_m"]

    @staticmethod
    def _unplaced_m(ctx):
        P = ctx.P
        return math.sqrt(max(float(P[E.PE, E.PE] + P[E.PN, E.PN]), 0.0))

    def _at_rest(self, mem, ts, e, n):
        """The receiver's last fixes lie within 3 m of each other over at least 3 s (a receiver with few satellites
        keeps moving for seconds after the car has stopped: such a fix is not where the car stands)."""
        q = mem.setdefault("rest", [])
        q.append((ts, e, n))
        while q and ts - q[0][0] > 6.0:
            q.pop(0)
        return ts - q[0][0] >= 3.0 and max(math.hypot(e - a[1], n - a[2]) for a in q) <= 3.0

    def _heading_check(self, ctx, mem, ts, sp):
        """The direction the receiver moves in (its own track, carried to now by the car's yaw) against the
        estimate's heading, while driving forwards. A heading that is the other way round, or a right angle off,
        is beyond what position rows correct through a linearisation: the heading is placed anew."""
        integ = ctx.integ
        n = mem.get("dir_n", 0)
        if not integ.anchored or getattr(integ, "_reanchor_pending", False) or sp[0] < 2.0 or n == mem.get("flip_seen"):
            return
        mem["flip_seen"] = n
        h = self._heading_now(ctx, mem, ts)
        ref = mem.get("dir")
        # (only a receiver that moves as the car does, and only while the car has not turned since that piece of
        # track: a receiver that trails by more than its delay shows the heading of before the turn)
        turned = integ.yaw_between(ref[0], integ.now) if ref is not None and integ.now is not None else None
        if h is None or turned is None or abs(turned) > 0.3 or mem.get("wander", 0.0) > 0.0:
            mem["flip"] = 0
            return
        off = abs((h - float(ctx.x[E.PSI]) + math.pi) % (2.0 * math.pi) - math.pi)
        mem["flip"] = mem.get("flip", 0) + 1 if off > self.flip_rad else 0
        if mem["flip"] >= self.flip_n:
            mem["flip"] = 0
            mem["flips"] = mem.get("flips", 0) + 1
            integ._reanchor("the heading is %.0f degrees off the direction the receiver moves in: placed anew" % math.degrees(off))

    def _held(self, ctx, mem, ts, jm):
        """A receiver whose held-back steps have grown beyond `hold_m` is somewhere else than the car (mirrored
        coordinates, a position jump of hundreds of metres): no rows, until its raw fixes lie within the
        estimate's reach again."""
        integ = ctx.integ
        if not mem.get("held") and jm > self.hold_m and integ.anchored:
            mem["held"] = True
            mem["held_n"] = 0
            mem["holds"] = mem.get("holds", 0) + 1
            integ.fdi.event("gnss", "held", None, "the receiver's positions have moved %.0f m in steps the car did not make: "
                            "not used until they return to the estimate (dead reckoning and the map meanwhile)" % jm,
                            quantity="position")
        if not mem.get("held"):
            return False
        ok = False
        prev = mem.get("prev")
        if prev is not None and integ.anchored:
            x, P = ctx.x, ctx.P
            reach = 3.0 * math.sqrt(max(float(P[E.PE, E.PE] + P[E.PN, E.PN]), 0.0)) + 3.0 * self.level(mem) * self.min_sigma_m + 20.0
            ok = math.hypot(prev[1] - float(x[E.PE]), prev[2] - float(x[E.PN])) <= reach
        mem["held_n"] = mem.get("held_n", 0) + 1 if ok else 0
        P = ctx.P
        ran_out = math.sqrt(max(float(P[E.PE, E.PE] + P[E.PN, E.PN]), 0.0)) > integ.cfg["position_max_hacc_m"]
        if mem["held_n"] >= self.release_n or not integ.anchored or (ran_out and mem.get("ok_run", 0) >= self.rejoin_n):
            mem["held"] = False
            mem["jump"] = None
            if mem["held_n"] >= self.release_n:
                integ.fdi.event("gnss", "released", None, "the receiver's positions are back at the estimate: used again",
                                quantity="position")
            elif integ.anchored:
                integ._reanchor("dead reckoning has run out and the receiver moves as the car does again")
            return False
        return True

    @staticmethod
    def _alive(ts):
        return [Meas("signal:gnss_fix", ts, True, role="signal")]

    def on_reanchor(self, ctx):
        """The position was declared lost and is anchored again on this receiver: whether the old position or
        the new fixes were wrong is not known, so the fixes are believed at their widest for a while."""
        ctx.mem["scale2"] = self.scale_max ** 2
        ctx.mem["scale_k"] = 0

    def calib_state(self, ctx):
        m = ctx.mem
        sb = m.get("sigma_b")
        if sb is None:
            return {}
        x = ctx.x
        return {"bias_sigma_m": round(sb, 1), "bias_e_m": round(sb * float(x[self.ix["ue"]]), 1),
                "bias_n_m": round(sb * float(x[self.ix["un"]]), 1), "level": round(self.level(m), 2),
                "level_measured": "level" in m}

    def report(self, ctx):
        m = ctx.mem
        n = m.get("n", 0)
        J = m.get("jump")
        why = None
        if m.get("held"):
            why = "the receiver's positions are not where the car can be (steps the car did not make): not used"
        elif J:
            why = "a step of %.0f m in the receiver's positions is held back" % math.hypot(*J) if math.hypot(*J) >= 5.0 else None
        elif m.get("wander", 0.0) > 0.0:
            why = "the receiver does not move as the car does (%.1f m/s unexplained): its error is taken to be growing" % m["wander"]
        return {"fix": m.get("fix"), "hacc_m": m.get("hacc"), "receiver": m.get("receiver"),
                "repeat_share": round(m.get("rep", 0) / n, 2) if n else None,
                "noise_scale": round(math.sqrt(min(max(m.get("scale2", 1.0), m.get("scale_u2", 1.0), 1.0), self.scale_max ** 2)), 2),
                "bias_sigma_m": None if m.get("sigma_b") is None else round(m["sigma_b"], 1),
                "wander_mps": round(m.get("wander", 0.0), 2), "jumps": m.get("jumps", 0), "delay_by_motion": m.get("lag_raw"),
                "held_back_m": round(math.hypot(*J), 1) if J else 0.0, "held": bool(m.get("held")), "few_satellites": bool(m.get("few")),
                "anchored": ctx.integ.anchored, "suspect_reason": why}


class GnssSpeedSource(Source):
    """The receiver's speed field: a witness (reported and compared, never fused, no vote)."""
    id = "gnss_speed"
    group = "gnss_speed"
    rank = 0
    family = "gnss"
    types = ("gnss",)
    stale_s = 5.0
    witness = True
    info = "receiver speed field (a delayed, smoothed output: witness only)"

    def measure(self, msg, ctx):
        if _repeat(msg, ctx.mem) or msg.get("fix") not in FIXES:
            return []
        v = msg.get("speed_mps")
        if not fin(v):
            return []
        acc = msg.get("speed_acc_mps")
        return [Meas("speed", msg["ts_mono_ns"] / NS, float(v), max(float(acc), 0.5) if fin(acc) else 1.5,
                     src="gnss.speed", role="monitor")]


class GnssTrackSource(Source):
    """The receiver's track as a speed: path length over a few fixes divided by their time. Slow (one value
    every few seconds, seconds old) and noisy, but independent of every speed sensor of the car and with no
    scale of its own: the third opinion that decides between the vehicle's speed and the camera's when they
    disagree (a vote), and a check on both when they agree. Never fused."""
    id = "gnss_track"
    group = "gnss_track"
    rank = 0
    family = "gnss"
    types = ("gnss",)
    stale_s = 1e9                # it speaks only while the car moves: its silence says nothing
    info = "receiver track speed (path over a few fixes): the independent third opinion on the speed"
    chord_n = 3                  # fixes per value
    min_speed, max_accel = 3.0, 1.5
    max_gap_s = 2.5
    # compared by its mean, over a long window (monitors.Pair mode "mean")
    pair = dict(mode="mean", max_step_s=8.0, pending_s=8.0, fire_window_s=60.0, recover_window_s=60.0, min_n=4,
                cusum_k=1.0, cusum_h=5.0, min_delta=0.06, recover_n=4, recover_z=2.5)

    def reset(self):
        pass

    def measure(self, msg, ctx):
        mem = ctx.mem
        lat, lon = msg.get("lat"), msg.get("lon")
        if msg.get("fix") not in FIXES or not fin(lat) or not fin(lon) or ctx.integ.origin is None:
            return []
        ts = msg["ts_mono_ns"] / NS
        e, n = E.to_local(ctx.integ.origin, float(lat), float(lon))
        pts = mem.setdefault("pts", [])
        if pts and (ts - pts[-1][0] > self.max_gap_s or ts <= pts[-1][0]):
            del pts[:]
        if pts and e == pts[-1][1] and n == pts[-1][2]:
            del pts[:]                                   # a repeated position: the receiver holds (standing, or no fix)
            return []
        pts.append((ts, e, n))
        if len(pts) <= self.chord_n:
            return []
        d = sum(math.hypot(pts[i + 1][1] - pts[i][1], pts[i + 1][2] - pts[i][2]) for i in range(len(pts) - 1))
        T = pts[-1][0] - pts[0][0]
        mid = 0.5 * (pts[-1][0] + pts[0][0])
        del pts[:-1]
        i = ctx.integ.model.index.get("gnss.tau")
        tau = float(ctx.x[i]) if i is not None else 0.0
        # the value belongs to the middle of the chord, minus the receiver's delay. While that delay is poorly
        # known and the car accelerates, the time the value belongs to is not known well enough to compare it
        # with anything: no evidence then, rather than evidence with a timing error in it
        s_tau = max(math.sqrt(max(float(ctx.P[i, i]), 0.0)) if i is not None else 0.5, 0.15)
        a0, a1 = ctx.ego_at(mid - tau - 0.5 * T), ctx.ego_at(mid - tau + 0.5 * T)
        if not (a0[3] and a1[3]) or abs(a1[0] - a0[0]) / T * s_tau > 0.35:
            return []
        # ... and a chord that contains a start, a stop or hard braking is not the speed at its middle
        if min(abs(a0[0]), abs(a1[0]), d / T) < self.min_speed or abs(a1[0] - a0[0]) / T > self.max_accel:
            return []
        hacc = msg.get("hacc_m")
        sig = 1.4 * min(max(float(hacc), 1.0) if fin(hacc) else 5.0, 8.0) / T
        return [Meas("speed", mid - tau, d / T, sig, src="gnss_track.speed", role="monitor")]
