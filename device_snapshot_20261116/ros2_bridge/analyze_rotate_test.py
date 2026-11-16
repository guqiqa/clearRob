#!/usr/bin/env python3
"""分析原地转向测试 CSV（record_state_odom.py 输出）。

按 /chassis/state 的角速度（|w|>0.05 rad/s）切分运动段，输出每段：
yaw 起止/变化、四轮 rpm 均值（检查左右反向）、state 角速度均值。
用法: analyze_rotate_test.py CSV
"""
import csv
import sys


def main() -> int:
    path = sys.argv[1]
    rows = []
    with open(path, newline="") as handle:
        for raw in csv.DictReader(handle):
            try:
                row = {k: float(v) for k, v in raw.items()}
            except (TypeError, ValueError):
                continue
            if "w_deg" in row and "w" not in row:
                row["w"] = row["w_deg"]  # 列名历史遗留，实际是 state.angular_velocity (rad/s)
            rows.append(row)
    if len(rows) < 10:
        print("NO_DATA rows=%d" % len(rows))
        return 1

    segments = []
    current = []
    for row in rows:
        if abs(row["w"]) > 0.05:
            current.append(row)
        else:
            if len(current) >= 5:
                segments.append(current)
            current = []
    if len(current) >= 5:
        segments.append(current)

    print("rows=%d segments=%d" % (len(rows), len(segments)))
    for index, seg in enumerate(segments):
        y0 = seg[0]["yaw_deg"]
        y1 = seg[-1]["yaw_deg"]
        dyaw = ((y1 - y0 + 180.0) % 360.0) - 180.0
        fl = sum(r["fl"] for r in seg) / len(seg)
        rl = sum(r["rl"] for r in seg) / len(seg)
        fr = sum(r["fr"] for r in seg) / len(seg)
        rr = sum(r["rr"] for r in seg) / len(seg)
        left = (fl + rl) / 2.0
        right = (fr + rr) / 2.0
        wmean = sum(r["w"] for r in seg) / len(seg)
        vmean = sum(r["v"] for r in seg) / len(seg)
        print("seg%d n=%d yaw=%.2f->%.2f dyaw=%+.2f deg w_state=%+.3f rad/s v=%.4f "
              "fl=%.2f rl=%.2f fr=%.2f rr=%.2f L=%.2f R=%.2f (反向=%s)"
              % (index + 1, len(seg), y0, y1, dyaw, wmean, vmean,
                 fl, rl, fr, rr, left, right,
                 "yes" if left * right < 0 else "no"))
    if not segments:
        print("NO_ROTATE_SEGMENT (state |w| 从未超过 0.05 rad/s)")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
