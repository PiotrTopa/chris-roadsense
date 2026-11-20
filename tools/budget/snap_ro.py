#!/usr/bin/env python3
"""snap_ro.py SNAP.txt : table from tools/budget/snap_ro.sh's output: CPU per process and thread in percent of
one core, split into little (cpu0-3) and big (cpu4-7) cores by the core the thread was last seen on, and
voluntary context switches (wake-ups) per second."""
import collections
import sys

pid, A, B, cur, T = {}, {}, {}, None, None
for line in open(sys.argv[1]):
    w = line.split()
    if not w:
        continue
    if w[0] == "PID":
        pid[w[1]] = " ".join(w[2:5])
    elif w[0] == "T":
        T = w
    elif w[0] in ("A", "B") and len(w) == 1:
        cur = w[0]
    elif cur and len(w) >= 7:
        (A if cur == "A" else B)[(w[0], w[1])] = (w[2], int(w[3]), int(w[4]), int(w[5]), int(w[6]))
dt, hz = float(T[2]) - float(T[1]), float(T[3])
print("%.1f s, hottest zone %s -> %s mC" % (dt, T[4], T[5]))
proc = collections.defaultdict(lambda: [0.0, 0.0, 0.0, collections.defaultdict(lambda: [0.0, 0.0, None])])
for k, (comm, j, cpu, vol, _inv) in B.items():
    if k not in A:
        continue
    d = (j - A[k][1]) / hz / dt * 100
    p = proc[k[0]]
    p[1 if cpu >= 4 else 0] += d
    p[2] += (vol - A[k][3]) / dt
    t = p[3][comm]
    t[0] += d
    t[1] += (vol - A[k][3]) / dt
    t[2] = cpu
tl = tb = 0.0
for p, (li, bi, wk, th) in sorted(proc.items(), key=lambda x: -x[1][0] - x[1][1]):
    tl, tb = tl + li, tb + bi
    print("%-52s little %6.1f  big %6.1f  wake-ups/s %6.0f" % (pid.get(p, "?")[:52], li, bi, wk))
    for c, (d, w, cpu) in sorted(th.items(), key=lambda x: -x[1][0]):
        if d >= 0.5 or w >= 20:
            print("    %-20s %6.1f %%  cpu%d  wake-ups/s %6.0f" % (c, d, cpu, w))
print("sum: little %.0f %% of 400, big %.0f %% of 400" % (tl, tb))
