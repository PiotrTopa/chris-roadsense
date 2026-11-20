# L47: the position / map loop on poor reception, and an honest position uncertainty

Lane of 2026-11-20, branch `v2-l47` from `main` 6054373. Laptop only (the phone was not touched). Owned paths:
the estimator's receiver and map rows (`chris_integrator/sources/gnss.py`, `sources/mapmatch.py`), `chris-mapper/`.
Edits outside them are listed in section 8. Reproducing commands: section 9.

Real evidence is four drives of one day with one receiver (20261119-104128, -113201, -151146, -153636). There is
no ground truth on them. Where a number rests on that, it says so; the claims that need truth (is the right road
matched, is the stated uncertainty right) are carried by simulation.

## 1. Result

| | main | this branch | legacy mapd |
|---|---|---|---|
| 113201: held-out fixes against the position 2 s before their stamp, p50 / p90 | 63.0 / 136.1 m | 34.9 / 66.2 m | 24.1 / 69.1 m |
| 113201: map available while moving | 83.5 % | 85.9 % | 93.3 % |
| 113201: receiver delay state while moving | 1.18 .. 4.67 s | 2.00 .. 2.32 s | - |
| the other three drives: held-out fixes p50 / p90 | 19.5 / 71.3, 14.1 / 27.3, 11.0 / 18.1 m | 23.9 / 67.6, 16.8 / 31.2, 9.6 / 18.2 m | 28.8 / 57.4, 15.9 / 35.4, 11.5 / 28.5 m |
| ... map available | 97.8, 98.5, 97.0 % | 97.2, 99.7, 100 % | 98.2, 98.6, 100 % |
| stated position uncertainty on the car: normalised error of held-out fixes (2 expected), four drives | 9.9, 18.5, 4.7, 2.8 | 1.1, 1.8, 4.1, 2.3 | - |
| ... share of held-out fixes inside the stated 95 % ellipse | 0.53, 0.71, 0.68, 0.86 | 0.98, 0.95, 0.90, 0.89 | - |
| simulation with truth, receiver like the car's (10 m slowly varying behind a claimed 4 m): position NEES (2 expected), town / motorway, 3 seeds | 6.3, 9.0, 22.8 / 6.9, 21.2, 27.6 | 2.3, 3.6, 0.5 / 1.3, 3.1, 2.0 | - |
| simulated fault table (59 faults, default modes): seconds an output was valid and confidently wrong | 1355 (position 655) | 1198 (position 493) | - |
| fault table on the recording slice (59 faults, default modes) | 1493 s (position 754, heading 234) | 1059 s (position 375, heading 178) | - |

Not reached: on 113201 the map is available 85.9 % of the driving against legacy's 93.3 %, and the held-out
fixes are 35 m from the position against legacy's 24 m (section 4.1 says what that measure rewards). On the two
drives with good reception the stated uncertainty is still 1.1-1.4 times too small in sigma (normalised 2.3 and
4.1 where 2 is expected; the median is right, the tail is not). Everything else that is still wrong: section 7.

## 2. Diagnosis

What "poor reception" is on this car, from the recordings (sections 9.1, 9.2):

1. **The whole input stops for 5-13 s at a time**: camera frames, vehicle samples and (mostly) fixes together,
   20 / 9 / 5 / 0 times on 104128 / 113201 / 151146 / 153636. 113201 has 9 such gaps in its 5 minutes of
   driving, and the car turns inside some of them. Legacy mapd keeps publishing through them (it coasts on the
   last motion and snaps to the next fix); main's estimator comes out of a gap with a heading it does not
   have: its white-jerk model states hundreds of metres, the first position rows are applied through a
   linearisation about the old heading, and the position is wrong by 100-260 m until the innovation monitor
   re-anchors it 8 s or more later (113201 at 341-372 s and 380-415 s: main's track leaves the road network
   and runs along roads the car was not on).
2. **The receiver's error is slowly varying, not white.** From fix to fix its steps agree with the odometer
   to 0.75-1.55 m (robust sigma); the fixes lie 7-10 m (median) beside the road, the same way for 5-20 s
   (autocorrelation of the cross-track offset 0.8-1.0 at 1 s, 0.25-0.87 at 5 s). Main fuses that as white noise
   of `hacc x 1..8` plus a 2.5 m correlated state: twenty fixes average to a quarter of the real error. That is
   the whole of "stated uncertainty optimistic by 5-7" (normalised 2.8-18.5 by this lane's tool), and of "two
   runs anchored on different fixes differ by more than 3 sigma" (the anchor took one fix's error as known to
   `2 hacc`, uncorrelated with the fixes that follow).
3. **With 3-5 satellites the receiver reports a smoothed track.** Both morning drives (104128 and 113201) have
   3-5 satellites throughout; the afternoon drives 5-10. On 113201 the receiver's positions keep moving at
   4-7 m/s for 13 s after the wheels have stopped (393-406 s), and crawl at 3 m/s while the car does 10; the lag
   at which its step lengths fit the odometer is 7.25 s (1.75-2.5 s on the other three drives). On 104128 there
   are stretches where it moves 5 m in 2 s while the car drives 30, and catches up 20 s later: its error
   reaches 100-190 m. So the delay is not a constant of the device, the walk of main's delay state towards
   4.6 s on 113201 was the filter following something real with the wrong tool, and "who predicts the held-out
   fixes" rewards whoever trails with the receiver.
4. **The loop.** At constant speed the delay and the along-track position are one quantity and the map says
   nothing along the road. With the map's rows stiff across the road (1 m per match, every 0.45 s, the choice
   of road taken as certain) and the receiver's error modelled as white, every along-track residual of a
   trailing receiver goes into the delay state: nothing in the loop measures the delay. And the match is
   computed from the pose it then corrects: a match with share >= 0.7 pulls the pose onto its line, which
   makes that line fit better (distance to the matched road 2 m through the box, 12 m for legacy; confidence
   0.74 against 0.56): the matcher's confidence is the pose's own distance returned. Each match is counted
   as independent of the last although its error (which road) is the same from one to the next.
5. **`map.conflict` at 12 m** is 1 sigma of this receiver with few satellites: it set the map aside for 16-21 %
   of healthy driving (L43).

Found in this lane's own intermediate builds, on the recordings and in simulation; listed because each is a
rule that reads as reasonable (several are main's) and each cost hundreds of metres once the estimator placed
headings more often than main does:

* A heading from the chord between two fixes (main's rule when there is no course) while the car reverses:
  104128, a three-point turn at 930-960 s, heading 180 degrees wrong, 700 m in 44 s.
* A heading from the receiver's course field (main's rule) right after a gap in which the car had turned: the
  receiver still reports the old course, steadily (104128 at 1047 s, 151146 at 667 s: 300-340 m).
* A heading from one step of a receiver that had just been standing and was settling (153636 at 25 s).
* The road's bearing as the first row after a gap, in simulation: a road is a line, and the row locked the
  heading the other way round with 8 degrees stated.
* A conflict limit of 12 m with the receiver's error modelled at its real size, in simulation: a correct map
  was taken away from a receiver that was 30 m off, and the position left the road by 48 m.
* A car park: at walking pace in a loop (153636, 440-466 s) map rows of any match that fits pulled the position
  onto the streets around the car park, 60-75 m off with 2.2 m stated.
* The receiver's own motion judged against the estimator's heading instead of its own track: with a wrong
  heading the healthy receiver was the one held back.

Item 3 of the brief (the end-to-end slice 104128 from 900 s for 240 s): the installed daemon starts cold at
900 s. Replayed offline the same way (`mixer_equiv.py run --start 900 --seconds 240 --mapper box`), main gives
`fused.map.available` 56.0 % (L43 measured 53-57 % end to end); the 44 % that are missing are 14.2 % no position
(36 s until the first anchor: the car stands, then manoeuvres at low speed, and main anchors only from a course
at 3 m/s or more), 20.0 % low match confidence, 4.7 % off road, 4.4 % position unknown after the three gaps of
all input in the slice, 0.7 % stale. Over the whole drive, warm, the same slice is at 74.2 % (the recorded
legacy map: 29.9 %). So: a cold start that waits for a course, and the gaps. This branch: 63.8 % (section 4.2).

## 3. The design change

Position is still estimated once, in the one EKF; the map box still reads the ego estimate only and returns
geometry. What changed is what each row is taken to know (`chris-integrator/docs/DESIGN.md` sections 3a, 4a,
15a have the full text).

**The receiver** (`sources/gnss.py`).

* A fix is `position(t - delay) + slowly varying error + 1.5 m white`. The slowly varying error is two
  Gauss-Markov states (20 s) in units of the receiver's present level `sigma_b = level x hacc x scale`, entering
  the rows multiplied by `sigma_b` (so the level can change from fix to fix without touching the filter's
  process model). `level` is measured where there is a map: the distance of raw fixes from the line of a
  confidently matched single road on straights, robustly, over 60 fixes (prior 2). `scale` (1-2.5) is the
  realised second moment of those states. An anchor carries that error too, fully anti-correlated with the
  position placed on it.
* **The delay is measured outside the loop**: the lag at which the receiver's step lengths fit the distances
  the car's speed sensors measured, over the last 60 steps, is a row on the delay state with a sigma from how
  sharply the steps determine it (nothing while the speed hardly varied). No position, no heading, no map
  enters it. With 5 satellites or fewer the delay's uncertainty is opened to 1.5 s until that measurement
  narrows it. The state's bounds are 0-9 s (the filter keeps 10 s of history); what bounds it in practice is
  that it is now informed. Position rows carry the delay's uncertainty and teach it only while the receiver
  moves as the car does.
* **The receiver's own motion against the car's**, raw against raw, the estimate not asked: the step from
  fix to fix against the distance driven and the direction of the receiver's own earlier track turned by
  the car's yaw. Two steps in three off by more than 2.5 m/s: unsteady; what it moves beyond the car's own
  motion is held back from its fixes (decaying over 45 s) and counted into `sigma_b`. One step beyond 8 m +
  half the car's step: a jump, held back the same way, and the position's covariance takes the step in (the
  receiver may have returned to the truth). More than 250 m held back: not used until its fixes return. Not
  while the car's own speed sources disagree.
* **Headings**: from the receiver's own track (two pieces in a row that agree, driven forwards, none longer
  than the car drove) carried to now by the car's yaw; the course field only if the yaw sensors were alive
  since the time it describes and the car has not turned since; never the chord. A heading 57 degrees off the
  direction a steady receiver moves in, three times running on a straight, is placed anew.
* A position that is not placed is put on a fix while the car stands (heading unknown, said so).

**Gaps** (`core/integrator.py`). More than 1.5 s without any measurement of speed and yaw rate: no position or
heading row whose time lies in the gap is used; when the car's motion is measured again and it moves, the
position is stated to `0.5 T^2 + 0.5 v T` (at least 40 m), put on the next fix, and the heading follows from
the receiver's track once that was driven after the gap. `Integrator.yaw_between` / `dist_between` give the
heading change and distance the car's own sensors measured between two times (None across a gap).

**The map's rows** (`sources/mapmatch.py`, `chris_mapper/matcher.py`).

* The match carries the other candidates' lines with their shares (`match.weight`, `match.alternatives`: from
  the fit alone with a mild preference for continuity), `fit` and `bearing_span_deg`. `confidence` and
  `ambiguous` are unchanged (the context's consumers read those).
* The cross-track row targets the share-weighted line of the winner and the candidates parallel to it; the
  spread of those lines multiplies a state of unit variance that the map's rows carry and never correct (a
  consider state). However many matches agree, the position across the road stays as uncertain as the choice
  between the roads. Candidates of another direction take weight from the row. No row while the car turns
  faster than 0.2 rad/s or its heading is more than 35 degrees off the road's direction.
* The heading row's sigma grows with the road's change of direction around the match; none while the
  heading is known worse than 20 degrees (which way along the line has to be known already).
* `map.conflict` is a matter of odds: how likely the offset of the receiver's recent fixes from every offered
  line is for the receiver's present error, against how likely a wrong match is at all (3 %, anywhere within
  60 m either side). Above even odds the map's rows are set aside; below, "the receiver is right" is one more
  line in the spread.

Along the road the map still says nothing, except at turns, where the new piece's line constrains what was
along-track before: that needs only an honest covariance along the track before the turn.

**Why this and not something else.** (1) *A second position filter without the map's rows*, to give the
matcher a pose the map has not touched: exact, but the position would be estimated twice, against the ruling
the component is built on. (2) *Covariance intersection* of the map row with the state: it assumes nothing
about the correlation, and so throws away what is known about it (it is the choice of road, constant between
junctions); the consider state says exactly that. (3) *A matcher that carries several road hypotheses through
time* (an HMM over raw fixes and odometry): the right matcher for a pose that cannot be trusted and the next
step if the shares prove too crude at junctions; it needs the fixes inside the map box, which today reads the
ego estimate only. (4) *Bounding the delay state and nothing else* (to 3.5 s): built first; in simulation with a
receiver that trails by 8 s it left the position 39 m behind at 9 times the stated variance. (5) *A model of
the receiver's smoothing* (a first-order lag as a state): one drive of 5 minutes shows that regime; the raw
comparison of displacements needs no model of it.

## 4. Numbers

### 4.1 The four drives (`tools/mapper_eval.py real`; every second fix held out of every system; car moving)

| drive (satellites) | system | map available | ambiguous | held-out fixes against the position 2 s before their stamp, p50 / p90 | ... at stamp minus the delay state | normalised by the stated uncertainty: mean (2 expected), median (1.39), inside the 95 % ellipse | delay state while moving, min .. max | re-anchorings |
|---|---|---|---|---|---|---|---|---|
| 113201 (3-5) | legacy | 93.3 % | 6.7 % | 24.1 / 69.1 m | | | | |
| | main | 83.5 % | 8.4 % | 63.0 / 136.1 | 54.7 / 139.6 | 9.94, 5.04, 0.53 | 1.18 .. 4.67 s | 3 |
| | this branch | 85.9 % | 27.5 % | 34.9 / 66.2 | 32.9 / 56.3 | 1.12, 0.67, 0.98 | 2.00 .. 2.32 | 6 |
| 104128 (3-5) | legacy | 98.2 % | 8.9 % | 28.8 / 57.4 | | | | |
| | main | 97.8 % | 9.5 % | 19.5 / 71.3 | 17.7 / 72.5 | 18.52, 2.91, 0.71 | 1.34 .. 2.00 | 8 |
| | this branch | 97.2 % | 7.8 % | 23.9 / 67.6 | 22.6 / 66.2 | 1.76, 0.92, 0.95 | 0.88 .. 4.41 | 5 |
| 151146 (5-10) | legacy | 98.6 % | 2.2 % | 15.9 / 35.4 | | | | |
| | main | 98.5 % | 7.0 % | 14.1 / 27.3 | 11.6 / 25.2 | 4.72, 2.41, 0.68 | 0.45 .. 2.23 | 5 |
| | this branch | 99.7 % | 4.8 % | 16.8 / 31.2 | 11.6 / 29.9 | 4.05, 1.07, 0.90 | 0.66 .. 2.00 | 5 |
| 153636 (5-10) | legacy | 100 % | 1.3 % | 11.5 / 28.5 | | | | |
| | main | 97.0 % | 0.2 % | 11.0 / 18.1 | 9.5 / 18.0 | 2.76, 2.13, 0.86 | 0.71 .. 2.44 | 1 |
| | this branch | 100 % | 7.7 % | 9.6 / 18.2 | 8.2 / 18.0 | 2.27, 1.20, 0.89 | 1.38 .. 2.44 | 2 |

How to read it.

* The held-out fixes are the same receiver's: the measure says who predicts the receiver, not who is nearer
  the road. Legacy takes fixes as the present position, so it is nearest them at lag 0 (4-19 m) and a
  receiver's delay behind the car. On 113201 the receiver trails by tens of metres more than its 2 s (section
  2.3); a position that does not trail with it is further from its fixes.
* The one check on a recording that needs no assumption about the delay is a **stop**: while the wheels stand
  the car is in one place, and a receiver that has come to rest by the end of the stop says where, to its own
  slowly varying error. Each system is asked where the car was 1 s into the stop. That is thin (stops of at
  least 6 s with the receiver at rest by the end: 1, 1, 8, 3 on the four drives):

  | drive | stops | legacy | main | this branch |
  |---|---|---|---|---|
  | 113201 | 1 | 68 m | 40 m | 48 m |
  | 104128 | 1 | 36 m | 21 m | 25 m |
  | 151146 | 8 | p50 6.2 m (2 4 1 6 5 28 10 12) | 8.3 m (3 2 9 4 38 25 6 8) | 6.9 m (8 6 6 7 51 9 6 7) |
  | 153636 | 3 | 5.1 m (3 7 5) | 7.9 m (3 9 8) | 5.8 m (5 6 13) |

  With few satellites both estimators are nearer the place the car stopped than legacy, on one stop each; with
  good reception all three are within the receiver's own error, and nothing separates them.
* 113201 moves from run to run of this lane's own variants (p50 23-39 m, availability 77-89 %): it has 43-53
  held-out fixes with the car moving and 9 gaps. The direction is firm (main's 63 m and its delay walk are gone
  in every variant); the second digit is not. Nothing was tuned on that drive alone: the receiver's noise floor,
  the jump limit and the unsteady limit were read off all four drives (section 9.2), the level is measured per
  drive, the satellite rule separates the two morning drives from the two afternoon ones.
* Availability on 113201 (85.9 % against legacy's 93.3 %): 11 % of the moving steps have no position (the
  seconds after each gap until a fix places it; legacy keeps the position it had and calls it dead reckoning),
  3 % are off road. The share marked ambiguous rose to 27.5 %: after a gap the pose is stated to tens of metres
  and the matcher sees several roads within that.
* The stated uncertainty: with few satellites it was 3-4 times too small in sigma and is now right (1.1, 1.8
  where 2 is expected; 95-98 % inside the 95 % ellipse). With good reception it was 1.2-1.5 times too small and
  the median is now right (1.07, 1.20 against 1.39) but the tail is not (mean 4.05 on 151146, where one stop is
  51 m off). Honest in the sense of "not optimistic" on two drives; nearly so on the other two.

### 4.2 The slice (item 3)

104128 from 900 s for 240 s, the stack started cold there, the map box in the loop, replayed offline
(`tools/mixer_equiv.py run --start 900 --seconds 240 --mapper box`; the installed daemon end to end was not run
in this lane). L43 measured 53-57 % end to end for main and 69 % for L39's layout.

| `fused.map` | main | this branch |
|---|---|---|
| available (the car side's `map.ok`) | 56.0 % | 63.8 % |
| not available: no position | 14.2 % | 2.4 % |
| ... position unknown (after the gaps of all input) | 4.4 % | 0.7 % |
| ... off road | 4.7 % | 0 |
| ... low match confidence | 20.0 % | 32.4 % |
| ... stale | 0.7 % | 0.7 % |
| `view.road.position.trusted` (the road picture's own gate) | 59.4 % | 60.8 % |

The cold start is shorter (the position is placed while the car still stands: no position for 5 s instead of
36 s) and the gaps cost less. What the position is worth in that first minute is another matter: it is placed
on a standing receiver (stated to 10-21 m), the car then creeps at walking pace (1 m/s in the median from 30 s
to 64 s), and a heading is not had until 74 s: the receiver's track gives no direction at that pace, and its
course field is not used below 3 m/s. A match without a heading does not reach confidence 0.5, so `fused`
reads `low_match_confidence` for 54 s of the first 64 s (it does not use such a match, rightly). L39's 69 % is
not reached. The road picture is level with main.

### 4.3 Simulation with truth

**The estimator alone** (`tools/integrator_eval.py sim`; the simulated map is always the right road). Position
rms (m) and NEES (2 degrees of freedom: 2 when the stated covariance is right), seeds 1 / 2 / 3. `good` is the
receiver the earlier lanes simulated (2 m slowly varying, declared a good one); `car` is like the reference
car's on a good day (10 m slowly varying and smooth from fix to fix, behind a claimed 4 m); `poor` adds gaps,
multipath jumps, repeated fixes and a track that trails by another 6 s (all seeded; `sim/sensors.py`):

| scenario, receiver | main: position rms | main: NEES | this branch: position rms | this branch: NEES | delay state at the end (truth 2.2 s; `poor`: 2.2 s and 6 s of smoothing) |
|---|---|---|---|---|---|
| town, good | 1.2 / 2.3 / 1.6 m | 0.36 / 1.23 / 0.78 | 1.7 / 1.6 / 2.2 m | 0.90 / 0.90 / 1.68 | 2.10 / 2.23 / 2.09 s |
| town, car | 7.1 / 9.8 / 16.9 | 6.3 / 9.0 / 22.8 | 5.8 / 9.0 / 4.2 | 2.3 / 3.6 / 0.5 | 1.92 / 2.33 / 2.08 |
| town, poor | 23.6 / 34.2 / 13.1 | 13.5 / 14.6 / 5.9 | 10.9 / 6.5 / 5.6 | 1.5 / 0.6 / 0.5 | 8.65 / 2.31 +- 1.5 / 5.09 (main: 5.8 / 4.9 / 5.8) |
| motorway, good | 4.3 / 6.8 / 9.6 | 0.74 / 2.01 / 3.96 | 2.3 / 5.8 / 6.9 | 0.71 / 1.29 / 1.15 | 2.21 / 2.30 / 2.34 |
| motorway, car | 15.0 / 29.4 / 47.0 | 6.9 / 21.2 / 27.6 | 10.6 / 18.5 / 16.6 | 1.3 / 3.1 / 2.0 | 2.32 / 2.59 / 2.67 |
| motorway, poor | 37.9 / 34.9 / 51.5 | 8.1 / 2.8 / 13.7 | 20.7 / 25.8 / 24.2 | 2.2 / 1.2 / 3.5 | 2.01 +- 1.5 / 2.39 +- 1.5 / 4.12 (main: 8.0 / 6.9 / 7.3) |

With the receiver the earlier lanes simulated nothing is lost (NEES 0.7-1.7 against 0.4-4.0). With a receiver
like the car's, main's stated variance is 6-28 times too small and its position 1.2-4 times worse; this
branch's NEES is 0.5-3.6. Speed, yaw rate, grade and lead columns are the same in both trees to the third
digit. Not right: the heading on the motorway with a car-like receiver (heading NEES 1.9-4.9, main 0.7-14.6:
too confident in both); and on the motorway with the poor receiver the delay is not found in two of three
seeds (the speed hardly varies: the state says 2 +- 1.5 s and the position trails with the receiver, 21-26 m).

**The loop on the real map** (`tools/mapper_eval.py synth`: the estimator with the map box in the loop,
simulated drives over the Poland pack with truth, fixes 2 s old, seeds 1 and 2 together; "sure and wrong" = a
road beside the route offered as an unambiguous match with confidence >= 0.5; honesty = error^2 against the
pose's own covariance, 2 expected):

| drive, receiver | on the driven route, of all steps: legacy / main / this branch | wrong road and sure of it: legacy / main / this branch | position error with a fix, p50 / p90: main; this branch | honesty (2 expected), share inside the 95 % ellipse: main -> this branch | delay state, min .. max (truth 2.0 s; `smooth`, `poor`: plus 6 s of smoothing): main; this branch |
|---|---|---|---|---|---|
| Krakow 4.2 km, good | 100 / 100 / 99.8 % | 0 / 0 / 0.08 % | 1.1 / 3.1; 1.6 / 3.5 m | 0.3, 1.00 -> 0.7, 1.00 | 1.8..2.3; 1.7..2.1 s |
| Krakow, car | 99.9 / 99.6 / 100 | 0 / 0.08 / 0 | 3.4 / 13.7; 1.7 / 5.0 | 3.9, 0.81 -> 0.6, 0.99 | 1.6..3.3; 1.8..2.5 |
| Krakow, gaps | 97.9 / 99.3 / 100 | 1.51 / 0.24 / 0 | 2.9 / 9.2; 1.3 / 3.9 | 2.3, 0.89 -> 0.4, 1.00 | 1.8..3.1; 1.8..2.7 |
| Krakow, jumps | 100 / 99.0 / 100 | 0 / 0.56 / 0 | 4.0 / 13.3; 1.7 / 6.0 | 2.4, 0.90 -> 0.9, 1.00 | 1.2..3.1; 1.8..2.6 |
| Krakow, repeats | 100 / 99.9 / 98.1 | 0 / 0 / 0.32 | 2.9 / 11.0; 1.1 / 3.8 | 2.0, 0.94 -> 0.1, 1.00 | 1.8..4.3; 1.6..2.8 |
| Krakow, smooth | 71.4 / 95.5 / 100 | 9.95 / 2.39 / 0 | 13.1 / 27.1; 1.5 / 5.5 | 22.6, 0.57 -> 0.4, 0.99 | 2.0..6.8; 1.7..7.4 |
| Krakow, poor | 72.0 / 91.7 / 94.8 | 5.89 / 2.31 / 0.08 | 12.8 / 40.4; 1.8 / 8.3 | 7.4, 0.64 -> 3.0, 0.95 | 2.0..7.3; 1.7..5.5 |
| Krakow, poor + host stalls | 77.0 / 63.0 / 76.5 | 4.55 / 6.04 / 3.24 | 12.1 / 97.6; 8.7 / 40.4 | 211, 0.59 -> 64, 0.87 | 2.0..7.4; 1.4..4.6 |
| Warsaw 5.4 km, good | 99.4 / 99.8 / 100 | 0.11 / 0 / 0 | 2.0 / 5.1; 1.4 / 3.5 | 0.9, 0.99 -> 0.6, 1.00 | -; 1.6..2.1 |
| Warsaw, car | 98.5 / 99.3 / 100 | 0.57 / 0.06 / 0 | 5.1 / 11.7; 1.3 / 5.7 | 4.5, 0.80 -> 0.6, 0.99 | -; 1.5..2.4 |
| Warsaw, gaps | 95.4 / 96.6 / 100 | 1.61 / 1.95 / 0 | 3.8 / 19.3; 1.0 / 3.7 | 40.6, 0.78 -> 0.2, 1.00 | -; 1.6..2.5 |
| Warsaw, jumps | 96.7 / 97.7 / 100 | 0.57 / 1.61 / 0 | 6.2 / 24.4; 1.3 / 4.8 | 43.0, 0.69 -> 0.4, 1.00 | -; 1.4..2.5 |
| Warsaw, repeats | 98.0 / 97.8 / 99.9 | 0.57 / 1.44 / 0 | 5.0 / 22.2; 0.9 / 3.1 | 39.5, 0.77 -> 0.5, 0.99 | -; 1.9..2.6 |
| Warsaw, smooth | 82.2 / 79.0 / 100 | 5.17 / 9.60 / 0 | 82.2 / 98.2; 2.1 / 23.4 | 352, 0.07 -> 2.8, 0.90 | -; 2.0..6.2 |
| Warsaw, poor | 76.1 / 78.0 / 90.2 | 6.67 / 11.78 / 2.53 | 70.4 / 101.6; 35.1 / 76.4 | 402, 0.14 -> 51, 0.60 | -; 1.5..3.7 |
| Warsaw, poor + host stalls | 72.4 / 53.7 / 69.6 | 4.52 / 22.14 / 7.36 | 75.1 / 662; 32.3 / 110 | 726, 0.01 -> 74, 0.57 | -; 1.7..5.5 |
| S2 with the tunnel 9.5 km, good | 99.6 / 100 / 100 | 0 / 0 / 0 | 3.1 / 5.8; 2.3 / 5.0 | 0.3, 1.00 -> 0.3, 1.00 | 1.8..2.1; 1.8..2.3 |
| S2, car | 98.2 / 97.3 / 100 | 0.84 / 2.03 / 0 | 11.2 / 21.8; 4.5 / 13.2 | 3.4, 0.86 -> 0.5, 1.00 | 1.6..2.5; 1.7..2.2 |
| S2, gaps | 97.1 / 99.9 / 100 | 2.24 / 0.07 / 0 | 10.9 / 22.1; 5.2 / 14.3 | 2.0, 0.91 -> 0.5, 1.00 | 1.8..2.9; 1.6..2.4 |
| S2, jumps | 92.9 / 94.5 / 100 | 4.90 / 2.73 / 0 | 9.1 / 23.6; 3.6 / 13.3 | 3.5, 0.86 -> 0.4, 0.99 | 1.8..2.7; 1.6..2.4 |
| S2, repeats | 100 / 100 / 93.0 | 0 / 0 / 0 | 8.8 / 24.8; 7.1 / 21.9 | 1.5, 0.94 -> 0.4, 1.00 | 1.6..3.3; 1.6..2.6 |
| S2, smooth | 97.4 / 98.5 / 100 | 1.82 / 0.35 / 0 | 22.9 / 49.2; 26.0 / 75.0 | 5.6, 0.70 -> 6.6, 0.70 | 1.9..7.2; 1.8..4.1 |
| S2, poor | 76.1 / 98.7 / 80.3 | 14.08 / 0.98 / 0 | 18.7 / 49.2; 24.8 / 110.7 | 5.1, 0.76 -> 9.2, 0.60 | 1.9..7.7; 1.1..3.6 |
| S2, poor + host stalls | 68.8 / 38.3 / 40.3 | 8.19 / 12.80 / 0.64 | 17.3 / 42.4; 65.0 / 95.4 | 23.0, 0.71 -> 7.1, 0.74 | 1.9..7.4; 1.9..2.4 |

(Legacy's position error with a fix is 25-28 m in town and 52-56 m on the S2 with every receiver, 84-204 m with
`smooth` and `poor`: it is a receiver's delay behind the car, and its matcher hides that on a straight road.)

What the simulation with truth carries:

* **The right road.** With a receiver like the car's, with gaps, jumps or a smoothed track, the box in the
  loop is on the driven route in 100 % of the steps on all three drives and never offers a wrong road as a sure
  match; main was sure of a wrong road in 0.06-9.6 % of the steps, legacy in 0-9.95 %. With everything at once
  (`poor`): 0-2.5 % (main 1.0-11.8 %, legacy 5.9-14.1 %); with the host stalling on top: 0.6-7.4 % (main 6.0-22.1 %,
  legacy 4.5-8.2 %).
* **Honesty.** With the car-like receiver and each single effect the stated covariance is now on the safe side
  (NEES 0.1-0.9 where 2 is expected: sigma about 1.5-2 times larger than the error; 99-100 % inside the 95 %
  ellipse) where main's was 2-43 times too small in variance. Not with everything at once: NEES 3-64 in Krakow,
  51-74 in Warsaw, 7-9 on the S2 (57-95 % inside the 95 % ellipse; main 1-76 %). The tail is the trailing
  receiver whose delay was not found: on those drives the delay state ends at 3.6-5.5 s where the receiver
  trails by 8 s.
* **What got worse.** On the S2 with the `poor` and `repeats` receivers the map is available in 80 % and 93 % of
  the steps against main's 99.7 % and 100 %: with few satellites the delay is stated to 1.5 s, which is 42 m at
  motorway speed, and in the 2.3 km tunnel dead reckoning then runs out of its 75 m sooner. And the position
  error with the `smooth` and `poor` receivers on the S2 is not better than main's (26 / 75 m against 23 / 49 m):
  at constant speed nothing shows how far the receiver trails.

**The loop on the hand-made network** (`chris-integrator/tests/test_map_loop.py`, asserted):

* Beside a twin road 8 m away for 1.2 km, receiver 10 m slowly varying: matched road the driven one in 100 % of
  the steps, the twin listed as a candidate in 100 %, never offered as a sure match; stated sigma across the
  road 3.2 m beside the twin against 2.0-2.5 m on the stretch without one; cross-track error p95 0.3-0.4 m.
* A 90 degree turn: stated east sigma 3.4-7.5 m before the turn (along the road), 2.2-2.4 m after it (across the
  new road); the east error itself 0.0-0.7 m after the turn.
* Few satellites and a host that stalls every 45 s for 6.5 s, 5.6 km: the road is the right one in 100 % of the
  available steps; valid and available 84-85 % (the rest is stated unknown after gaps); the position error is
  32-43 m (median) with a receiver that trails by up to 120 m there, and it is within 2.5 of its own stated
  sigmas in 70-83 % of the steps (confidently wrong, beyond 15 m and 3 sigmas, in 16 % on the asserted seed):
  better than the receiver, not honest in the tail.

### 4.4 The fault table

Simulation (town, seed 1, 59 faults from 95 s to 150 s, default detector modes; seconds an output was valid and
wrong beyond its tolerance and 3 of its own sigmas):

| | main | this branch |
|---|---|---|
| all 59 faults | 1355 s | 1198 s |
| ... position / heading / speed / yaw rate / lead | 655 / 119 / 424 / 56 / 102 | 493 / 124 / 424 / 56 / 102 |
| no output confidently wrong | 30 faults | 31 |
| receiver: position jump 30 m | position 41 s | 0 |
| receiver: slow drift 1 m/s | position 35 s | 0 |
| receiver: fixes 2 s older | position 33 s | position 3 s |
| receiver: lat / lon swapped | heading 18 s, position 50 s | heading 21 s, position 18 s |
| receiver: noise burst, outliers, frozen fix, lying accuracy, drop-out | 0 | 0 |
| map: parallel road 25 m / bearing turned 90 degrees | 0 / 0 | 0 / 0 |
| camera out of range and receiver jump | lead 2 s, position 41 s | lead 2 s |
| vehicle speed: stuck / sign | position 46 / 58 s, heading 0 / 12 s | position 29 / 28 s |
| **worse**: vehicle speed bias +2 m/s / scale 0.9 / slow scale drift to +17 % | position 39 / 40 / 13 s | position 48 / 50 / 36 s |
| **worse**: vehicle and camera speed x 0.7 | position 42 s | position 49 s |
| **worse**: gyro scale 1.3 / slow bias drift | heading 17 / 27 s, position 8 / 20 s | heading 23 / 27 s, position 19 / 23 s |
| **worse**: vehicle speed unit; camera yaw rate scale 1.5 | heading 0; 0 | heading 7 s; 4 s |

The rows that got worse are faults of the car's own speed or yaw rate that, with the default modes, stay in
the estimate: the receiver's rows now correct the position along the road more slowly, because its slowly
varying error is no longer taken for white noise. Speed, yaw rate and lead columns are identical to main's in
every row.

On the recording (104128, 260-560 s, fault 355-410 s, default modes; the reference is each tree's own unfaulted
run):

| | main | this branch |
|---|---|---|
| all 59 faults | 1493 s | 1059 s |
| ... position / heading / speed / yaw rate / lead | 754 / 234 / 405 / 39 / 61 | 375 / 178 / 405 / 39 / 61 |
| no output confidently wrong | 22 faults | 34 |
| receiver: jump 30 m / slow drift / frozen fix / lat-lon swapped / lying accuracy / drop-out / fixes 2 s older | position 12 / 30 / 46 / 50 / 4 / 38 / 43 s | 0 in all seven |
| map: bearing turned 90 degrees | position 39 s | 0 |
| **worse**: map: matcher on a parallel road 25 m away | position 12 s | position 30 s |
| camera mount: pitch step 1.7 degrees | heading 8 s, position 44 s | 0 |
| vehicle speed: sign / scale 0.9 / slow scale drift | position 64 / 30 / 14 s (sign: heading 18 s) | position 24 / 17 / 7 s |
| **worse**: vehicle speed: bias +2 m/s / scale 0.7; vehicle and camera x 0.7 | position 28 / 48; 51 s | position 33 / 56; 60 s |
| **worse**: gyro: bias 0.02 rad/s / slow bias drift / stuck | heading 25 / 22 / 19 s, position 17 / 18 / 11 s | heading 45 / 31 / 20 s, position 28 / 14 / 14 s |
| **worse**: camera yaw rate: sign | heading 7 s, position 6 s | position 16 s |

The receiver rows are at 0 here because the estimate's uncertainty now covers what the fault does (the measure
is "wrong beyond the tolerance and beyond 3 of its own sigmas") and, for the jump and the swap, because the step
is held back. L43's note that the position columns on the recording were noise from re-anchoring on different
fixes (a camera pitch step "moving the position" for 44 s) is gone with it: that row is 0. The parallel road is
the cost of `map.conflict` no longer firing at 12 m with a receiver whose error is 13-17 m: at 25 m the odds
say "the receiver, probably", the map's rows stay, and the position is on the wrong road for the rest of the
fault with 5 m stated. The gyro rows: a heading that drifts is corrected more slowly by a receiver whose rows
weigh less.

### 4.5 Nothing else moves (`tools/mixer_equiv.py run` in both trees, `stats`)

The four drives, each tree with the recorded map (the configuration of L43's regression) and with the box in
the loop; main's output against this branch's, frame by frame:

| | 104128 (28 288 frames) | 113201 (9746) | 151146 (17 720) | 153636 (7261) |
|---|---|---|---|---|
| `lead.valid`, `lead.present`, `lead.mode` differ | 0 frames | 0 | 0 | 0 |
| `lanes.valid` differs | 0 | 0 | 0 | 0 |
| `ego_state` speed or yaw rate validity differs | 0 | 0 | 0 | 0 |
| lead distance: frames differing; p50 / p95 / max | 0.3 %; 0.01 / 0.01 / 0.01 m | 1.1 %; 0.01 / 1.44 / 3.16 m | 0.0 %; 0.01 m | 6.4 %; 0.02 / 0.02 / 0.02 m |
| relative speed | 0.2 %; 0.01 / 0.01 / 0.05 m/s | 1.5 %; 0.05 / 2.3 / 4.9 | 0.1 %; 0.03 / 0.01 / 0.04 | 5.6 %; 0.05 / 0.1 / 0.12 |
| ego speed in `lead` | 1.7 %; 0.001 / 0.064 / 0.87 m/s | 3.0 %; 0.001 / 1.16 / 5.38 | 1.0 %; 0.001 / 0.001 / 0.037 | 1.4 %; 0.001 / 0.002 / 0.18 |
| ego yaw rate in `lanes` | 20 %; 0.0001 / 0.0013 rad/s | 20 %; 0.0003 / 0.0041 | 26 %; 0.0001 / 0.0025 | 13 %; 0.0003 / 0.001 |
| frames with speed off by > 0.3 m/s or yaw rate by > 0.03 rad/s | 34 | 82 | 0 | 0 |
| ... of which without a fresh vehicle sample in main's own frame | 34 | 82 | - | - |

(Recorded map; with the box the same rows are within the same ranges, lead distance on 113201 at most 0.09 m.)

Where from. (a) Position, heading and the receiver's states share one filter with the speed and the yaw
rate: other position rows move the yaw rate in the fourth decimal and the speed in the third (the median
differences). (b) Every larger difference lies in a frame without a fresh vehicle sample, in or right after a
gap of the input on the two morning drives: there the estimate coasts, this branch restarts the filter's
buffer when the gap ends (the position and heading are placed anew), and what both trees publish in such a
frame is a prediction; the lead distances that differ by metres on 113201 follow the ego speed of those
frames. On the two drives without such frames beyond the vehicle's own silences nothing differs by more than
0.18 m/s. (c) One frame on 113201 was published with speed and yaw rate `no_measurement` in a first build
(two restarts of the buffer in one message forgot when the speed was last measured); fixed in `core/ekf.py`
(`rebase` keeps what an earlier restart carried), and the table is from after that.

With the box in the loop `view.road` differs as intended: `road.position.hacc_m` by 1-23 m (median; it is the
stated uncertainty), junction and branch distances by 1-8 m (median) on three drives and by 90 m on 113201
(another position).

### 4.6 Tests

chris-integrator 351 pass (331 before; with the car repository present), chris-mapper 30 (22), roadsense api 22,
mapd 81. New, none needs a recording or a map pack: `chris-integrator/tests/test_position_loop.py` (17: the
receiver model against truth, the measured delay, a smoothed track, a jump, poor reception; a gap with a turn
inside; a position placed while standing; the map's hypotheses; mirrored coordinates, a jump and a drift;
`yaw_between` / `dist_between`), `tests/test_map_loop.py` (3: the whole loop on the hand-made network: a twin
road, a turn, few satellites with a stalling host), `chris-mapper/tests/test_box.py` (+8: the match's shares,
alternatives, fit and bearing span; the simulated receiver). One existing end-to-end test
(`test_vehicle_e2e.py`, a batch of fixes posted at once after a `reset`) failed on the first build and showed a
real hole: with nothing known about the car's motion before the first fix, no heading was ever placed. Fixed
(a steady course of a car that drives straight is taken when there is no position at all).

## 5. The receiver distrusted (item 4)

What is achievable with map + odometry + heading, and what is built, is in `chris-integrator/docs/FAULTS.md`
("The receiver distrusted"). In short: across the road the map; along the road the odometer, reset at turns of
the matched road; found out raw against raw; held back; re-acquired when the fixes return or when dead reckoning
has run out and the receiver moves as the car does again. In simulation with truth (receiver gone for 170 s,
1.7 km): error along the road at most 1.4 m, across 1.0 m, stated 2.2 m after each bend growing to 8-13 m before
the next; without the map 18-32 m, stated 31-44 m. Mirrored coordinates for 55 s: with a receiver like the
car's the position error stays under 25 m (`tests/test_position_loop.py`); in the fault campaign (the 2 m
receiver) the position is confidently wrong for 18 s, up to 318 m, against main's 50 s and 554 m: better, not
survived. What it cannot do: a slow drift along the road is followed; a step that stays is followed after the
best part of a minute; one absolute source.

## 6. Parallel roads nearer than 12 m (item 5)

A receiver cannot tell two roads apart that are nearer than about 3 of its own sigmas: 12 m for a 3 m
receiver, 35-40 m for the reference car's with 3-5 satellites. Below that the design does not detect, it
states: the matcher lists the other road as a candidate with its share, the row targets the share-weighted
line, and the position across the road is as uncertain as the choice (8 m apart at 60 : 40 is 3.9 m; section
4.3, the twin road). Where the map has no other road and the receiver still lies beside the matched one, the
odds that the match is wrong go into the same spread, and above even odds the map's rows are set aside.

## 7. Still wrong, or not verified

* **113201 is not at legacy's level** on the two measures of the brief (35 m against 24 m, 85.9 % against
  93.3 %), and its numbers move by a third between variants of this lane (section 4.1). What is firm: the
  position no longer leaves the road network, the delay state no longer walks, the uncertainty is no longer
  3-4 times too small.
* **The stated uncertainty with good reception** has a tail (normalised mean 4.1 and 2.3; medians right). The
  level is measured across the road only, on straights; along the road the receiver's lag adds error that the
  `scale` term catches only in part.
* **The measured delay is noisy**: on 104128 it moved between 0.9 and 4.4 s. In simulation with a receiver
  that trails by 8 s it lands at 5-8.6 s (a first-order lag is not a pure delay). One drive of 5 minutes shows
  that regime on the car.
* **Faults of the car's own speed** cost more position time than before with the default detector modes
  (section 4.4): the receiver corrects along the road more slowly.
* **After a receiver fault, alarms last as long as something is held back** (up to a minute after the fault).
* **A slow drift and a persistent offset of the receiver are followed** along the road (one absolute source).
* **The candidates' shares are a snapshot**, not a history: at junctions with several roads of similar
  direction the shares can flip from one match to the next. The hypothesis-carrying matcher (section 3,
  alternative 3) is the cure; not built.
* **Ambiguity rose** on two of four drives through the box's own `ambiguous` flag (113201: 27.5 % against 8.4 %;
  153636: 7.7 % against 0.2 %; it fell on the other two): the pose's stated uncertainty is wider, and the
  matcher's share falls with it. The map context's consumers (`fused`, the road picture) read `confidence`;
  what that does to them was measured on one slice only (section 4.2): more `low_match_confidence`.
* **A heading at walking pace** takes long (74 s on the cold slice): the receiver's track gives no direction
  below a few metres a step, and the map's bearing is not taken for a heading that is not known already.
* **Thresholds from four drives of one day**: the receiver's noise floor (2.5 m/s, two steps in three), the
  jump limit (8 m + half the car's step), the level's prior (2), the satellite count at which the delay is
  opened (5), the odds of a wrong match (3 %). They are class attributes; none was fitted to 113201 alone.
* **Not run**: nothing on the phone; the cost on the phone; the end-to-end run with the installed daemon
  (`tools/stack_e2e.py --installed`: the cold slice was replayed offline instead, which reproduces L43's end-to-end number for main, 56.0 % against 53-57 %); the car repository's own end-to-end tests.
* **Cost**: one more state (27 instead of 26), a few bisect lookups per fix (and 1260 every fifth fix for the
  delay measurement), one note per fused message for `yaw_between` / `dist_between`. Measured on the laptop:
  153636 with the box in the loop, both trees alternately, twice: 3.58 / 3.25 ms per road frame for main, 3.61 / 3.46 ms for this branch (the estimator 0.87 / 0.80 -> 0.87 / 0.84 ms per call, the map box 0.23 / 0.21 -> 0.24 / 0.23 ms per pose): 1-6 % more.

## 8. Files touched outside the owned paths

Lane L46 works on the core and the mixer at the same time; these are the edits it will meet:

| file | what | size |
|---|---|---|
| `chris_integrator/core/ego.py` | `EgoModel.set_clamp` (bounds of one calibration state at run time) | +7 lines, nothing existing changed |
| `chris_integrator/core/integrator.py` | `abs_position` (new optional parameters: the slowly varying error's coefficient, a held-back offset, a heading, placing without a heading), `_anchor` (the error's correlation; a position without a heading), `_reanchor` (a reason, a stated position sigma), new `_gap_reanchor`, `widen_position`, `_yaw_note`, `yaw_between`, `dist_between`; `_cache` calls `_yaw_note`; `_measure` drops pose rows across a gap; `on_message` acts on a gap; three config keys (`gap_s`, `heading_lost_rad`); `_POSE_ROWS` | about 200 lines added, 25 changed; the monitors, the object layer, the output and the mixer are untouched |
| `chris_integrator/core/ekf.py` | `Filter.rebase` keeps what an earlier restart carried (two restarts in one message forgot when the speed was last measured: one frame `no_measurement`) | +3 lines; the filter's run loop is untouched |
| `chris_integrator/sim/sensors.py` | `GnssSim`: few-satellite effects, a smooth slowly varying error (`_GM2`); defaults unchanged | +60 lines |
| `tools/mapper_eval.py`, `tools/integrator_eval.py`, `tools/mixer_equiv.py` | `--tree`, `--rx`, `--seeds`, stops, honesty; `--rx`; `--start` / `--seconds` | tools |
| `chris-integrator/docs/DESIGN.md`, `FAULTS.md`, `TOPICS.md` | sections 3a, 4a, 15a; "The receiver distrusted"; the new fields | docs |

The filter's inner loop is not touched; `_yaw_note` runs once per fused message (a dozen float operations).
The state grew by one (`gnss.ue` / `un` replace `be` / `bn`; `map.amb` is new).

## 9. Reproducing

```sh
cd ~/projects/wt/roadsense-l47; D=~/datasets/roadsense-rec; P=$D/trip-pos/pos_20261119.txt; O=/tmp/l47; mkdir -p $O
PY=~/projects/roadsense/.venv-models/bin/python; PP=$(python3 -c "import pytest,os;print(os.path.dirname(os.path.dirname(pytest.__file__)))")
# "main" below = a checkout of 6054373 with this branch's three tools and sim/sensors.py copied over it (the
# same tools, simulator and receivers on both sides):
BASE=~/projects/wt/roadsense-l47-base    # git worktree add --detach $BASE 6054373; cp tools/{mapper_eval,integrator_eval,mixer_equiv}.py $BASE/tools/; cp chris-integrator/chris_integrator/sim/sensors.py $BASE/chris-integrator/chris_integrator/sim/
# every command below ran inside: systemd-run --user --collect --wait -p MemoryMax=12G -E PYTHONHASHSEED=0 -E PYTHONDONTWRITEBYTECODE=1 -E PYTHON=$PY -- bash -c "..."
# tests (no recording, no map pack needed)
(cd chris-integrator && PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider tests)
(cd chris-mapper && PYTHONPATH=$PP $PY -m pytest -q -p no:cacheprovider tests)
# 9.1 the four drives: this branch, and main with the same tool (sections 1, 4.1)
for s in 113201 104128 151146 153636; do
  $PY tools/mapper_eval.py real $D/20261119-$s --pos $P > $O/real_$s.txt
  $PY tools/mapper_eval.py real $D/20261119-$s --pos $P --tree $BASE > $O/real_main_$s.txt; done
# 9.2 the receiver against the odometer (section 2): lag, chords, repeats; satellites per trip are in $P ("sats")
$PY tools/integrator_eval.py gnss $D/20261119-{104128,113201,151146,153636} --pos $P
# 9.3 the slice of item 3, started cold as the installed daemon is (sections 2, 4.2)
$PY tools/mixer_equiv.py run $D/20261119-104128 --pos $P --mapper box --start 900 --seconds 240 --out $O/slice.jsonl.gz
(cd $BASE && $PY tools/mixer_equiv.py run $D/20261119-104128 --pos $P --mapper box --start 900 --seconds 240 --out $O/slice_main.jsonl.gz)
# 9.4 simulation with truth: the estimator alone (section 4.3)
for rx in good car poor; do for sc in town highway; do
  $PY tools/integrator_eval.py sim --scenario $sc --rx $rx > $O/sim_${sc}_$rx.txt
  (cd $BASE && $PY tools/integrator_eval.py sim --scenario $sc --rx $rx > $O/sim_main_${sc}_$rx.txt); done; done
# 9.5 simulation with truth: the loop on the Poland pack (section 4.3)
for demo in krakow-city warsaw-city warsaw-s2-tunnel; do
  $PY tools/mapper_eval.py synth --demo $demo --lag 2.0 --seeds 1 2 --rx good car smooth jumps gaps repeats poor poor+blackouts > $O/synth_$demo.txt
  $PY tools/mapper_eval.py synth --demo $demo --lag 2.0 --seeds 1 2 --rx good car smooth jumps gaps repeats poor poor+blackouts --box-only --tree $BASE > $O/synth_main_$demo.txt; done
# 9.6 the fault table (section 4.4): simulation and the recording slice, both trees
$PY tools/fault_campaign.py sim -j 4 --out $O/fc_sim.json; (cd $BASE && $PY tools/fault_campaign.py sim -j 4 --out $O/fc_sim_main.json)
$PY tools/fault_campaign.py real $D/20261119-104128 --pos $P --from 260 --to 560 -j 4 --out $O/fc_real.json
(cd $BASE && $PY tools/fault_campaign.py real $D/20261119-104128 --pos $P --from 260 --to 560 -j 4 --out $O/fc_real_main.json)
$PY tools/fault_campaign.py show $O/fc_sim_main.json $O/fc_sim.json; $PY tools/fault_campaign.py show $O/fc_real_main.json $O/fc_real.json
# 9.7 nothing else moves (section 4.5): frame by frame, both trees, the recorded map and the box
for s in 104128 113201 151146 153636; do for m in recorded box; do
  $PY tools/mixer_equiv.py run $D/20261119-$s --pos $P --mapper $m --out $O/eq_${s}_$m.jsonl.gz
  (cd $BASE && $PY tools/mixer_equiv.py run $D/20261119-$s --pos $P --mapper $m --out $O/eq_main_${s}_$m.jsonl.gz)
  $PY tools/mixer_equiv.py stats $O/eq_main_${s}_$m.jsonl.gz $O/eq_${s}_$m.jsonl.gz --fields 40; done; done
```

The tables were produced by the committed code but for one change made after them: `Filter.rebase` keeping an
earlier restart's carry (section 4.5 (c)). After it, the tests and the frame comparison of 113201 were run
again; the four-drive, simulation and fault tables were not (the change touches one frame's validity flags).
113201 varies between builds by more than that (section 4.1).

The plots, the per-step traces and the episode lists the diagnosis was read from came from one-off scripts in
the lane's scratch directory (a track dump of both systems per drive, drawn over the map pack's roads; the
receiver's steps against the odometer; the list of gaps per stream); the numbers they gave are the ones in
section 2, and `tools/mapper_eval.py real` reproduces the tables.

## 10. Branch

`v2-l47`; head: see the lane's hand-back. Not pushed, not merged.
