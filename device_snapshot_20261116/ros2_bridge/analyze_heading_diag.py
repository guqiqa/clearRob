#!/usr/bin/env python3
import csv, sys, math
rows = list(csv.DictReader(open(sys.argv[1])))
if len(rows) < 20:
    print("NO_DATA rows=%d" % len(rows))
    raise SystemExit
xs = [float(r["x"]) for r in rows]
ys = [float(r["y"]) for r in rows]
yaws = [float(r["yaw_deg"]) for r in rows]
vs = [abs(float(r["v"])) for r in rows]
fl = [float(r["fl"]) for r in rows]
rl = [float(r["rl"]) for r in rows]
fr = [float(r["fr"]) for r in rows]
rr = [float(r["rr"]) for r in rows]
odom = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
print("odom_displacement=%.4f m" % odom)
print("yaw_start=%.2f yaw_end=%.2f drift=%.2f deg" % (yaws[0], yaws[-1], yaws[-1] - yaws[0]))
idx = [i for i, r in enumerate(rows) if abs(float(r["v"])) > 0.05]
if idx:
    def mean(v): return sum(v) / len(v)
    L = [(fl[i] + rl[i]) / 2.0 for i in idx]
    R = [(fr[i] + rr[i]) / 2.0 for i in idx]
    print("steady_n=%d left=%.2f right=%.2f diff_R_minus_L=%.2f rpm" % (len(idx), mean(L), mean(R), mean(R) - mean(L)))
    print("steady_fl=%.2f rl=%.2f fr=%.2f rr=%.2f" % (mean([fl[i] for i in idx]), mean([rl[i] for i in idx]), mean([fr[i] for i in idx]), mean([rr[i] for i in idx])))
    dv = (mean(R) - mean(L)) / 60.0 * 2 * math.pi * 0.097
    print("implied_yaw_rate_from_steady_diff=%.3f deg/s" % (math.degrees(dv / 0.45)))
mv = [i for i, r in enumerate(rows) if abs(float(r["v"])) > 0.01]
if mv:
    s = mv[0]; e = min(len(rows), s + 40)
    def mean2(v): return sum(v) / len(v)
    L2 = [(fl[i] + rl[i]) / 2.0 for i in range(s, e)]
    R2 = [(fr[i] + rr[i]) / 2.0 for i in range(s, e)]
    print("startup_first_rows n=%d left=%.2f right=%.2f diff_R_minus_L=%.2f rpm" % (e - s, mean2(L2), mean2(R2), mean2(R2) - mean2(L2)))
