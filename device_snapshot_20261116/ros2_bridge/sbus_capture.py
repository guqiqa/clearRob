#!/usr/bin/env python3
"""SBUS 通道变化捕捉：采集 N 秒，打印各通道 min/max/range 与变化通道。

用法: python3 sbus_capture.py [秒数] [设备]
默认 5 秒、/dev/ttyS5。运行后立即做动作，结束后看输出。
"""
import re
import select
import statistics
import subprocess
import sys
import time

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
DEV = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyS5"
MON = "/root/ros2_bridge/sbus_monitor"

proc = subprocess.Popen([MON, DEV], stdout=subprocess.PIPE,
                        stderr=subprocess.DEVNULL, text=True)
rows = []
t0 = time.time()
print("collecting %.1f s ..." % DUR, flush=True)
while time.time() - t0 < DUR:
    r, _, _ = select.select([proc.stdout], [], [], 0.2)
    if not r:
        continue
    line = proc.stdout.readline()
    if not line:
        break
    vals = [int(m.group(2)) for m in re.finditer(r"CH(\d+)=(\d+)\(", line)]
    if len(vals) >= 10:
        rows.append(vals)
proc.terminate()
try:
    proc.wait(timeout=2)
except Exception:
    proc.kill()

if not rows:
    print("NO_FRAMES")
    raise SystemExit(1)

n = min(len(r) for r in rows)
k = min(10, len(rows))
base = [int(statistics.median([r[i] for r in rows[:k]])) for i in range(n)]
print("frames=%d baseline: %s" % (len(rows),
      " ".join("CH%d=%d" % (i + 1, base[i]) for i in range(n))))
changed = []
for i in range(n):
    vals = [r[i] for r in rows]
    lo, hi = min(vals), max(vals)
    if hi - lo > 50 or abs(lo - base[i]) > 50 or abs(hi - base[i]) > 50:
        changed.append(i + 1)
        print("CH%-2d base=%4d min=%4d max=%4d range=%4d"
              % (i + 1, base[i], lo, hi, hi - lo))
print("CHANGED: " + (", ".join("CH%d" % c for c in changed) if changed else "NONE"))


def _runs(vals, tol=30):
    out = []
    for v in vals:
        if not out or abs(v - out[-1][0]) > tol:
            out.append([v, 1])
        else:
            out[-1][1] += 1
            out[-1][0] = int(round((out[-1][0] * (out[-1][1] - 1) + v) / out[-1][1]))
    return out


if changed:
    print("=== sequence (in order) ===")
    for i in range(n):
        if (i + 1) not in changed:
            continue
        seq = _runs([r[i] for r in rows])
        print("CH%-2d " % (i + 1) + " -> ".join("%d(x%d)" % (v, c) for v, c in seq))
