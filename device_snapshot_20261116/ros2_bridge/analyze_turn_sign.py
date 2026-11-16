#!/usr/bin/env python3
"""分析弧线/直线测试 CSV（record_state_odom.py 输出）。

按 |v|>0.02 m/s 切分运动段，输出每段：
- 时长、yaw 起止/变化、实际 yaw 角速度；
- /chassis/state 角速度均值（wheel 派生，rad/s 与 deg/s）；
- 四轮 rpm 均值与左右差；
- 实际 yaw 率 / state 角速度 的比值（转向效率）。
用法: analyze_turn_sign.py CSV
"""
import csv
import sys

MOTION_V_THRESHOLD = 0.02
RAD_TO_DEG = 57.29577951308232


def mean(values):
    return sum(values) / len(values) if values else 0.0


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
                row["w"] = row["w_deg"]  # 列名遗留：实际是 state.angular_velocity (rad/s)
            if "t" not in row:
                row["t"] = 0.0
            rows.append(row)
    if len(rows) < 10:
        print("NO_DATA rows=%d" % len(rows))
        return 1

    segments = []
    current = []
    for row in rows:
        if abs(row["v"]) > MOTION_V_THRESHOLD:
            current.append(row)
        else:
            if len(current) >= 5:
                segments.append(current)
            current = []
    if len(current) >= 5:
        segments.append(current)

    print("rows=%d segments=%d" % (len(rows), len(segments)))
    for index, seg in enumerate(segments):
        t0 = float(seg[0]["t"])
        t1 = float(seg[-1]["t"])
        duration = max(t1 - t0, 1e-6)
        y0 = seg[0]["yaw_deg"]
        y1 = seg[-1]["yaw_deg"]
        dyaw = ((y1 - y0 + 180.0) % 360.0) - 180.0
        rate = dyaw / duration
        fl = mean([r["fl"] for r in seg])
        rl = mean([r["rl"] for r in seg])
        fr = mean([r["fr"] for r in seg])
        rr = mean([r["rr"] for r in seg])
        left = (fl + rl) / 2.0
        right = (fr + rr) / 2.0
        w_rad = mean([r["w"] for r in seg])
        w_deg = w_rad * RAD_TO_DEG
        ratio = abs(rate) / abs(w_deg) if abs(w_deg) > 1e-6 else 0.0
        print(
            "seg%d n=%d dur=%.2fs yaw=%.2f->%.2f dyaw=%+.2fdeg yaw_rate=%+.2fdeg/s "
            "w_state=%+.3frad/s(%+.1fdeg/s) ratio=%.2f "
            "fl=%.2f rl=%.2f fr=%.2f rr=%.2f left=%.2f right=%.2f R-L=%+.2f rpm"
            % (index + 1, len(seg), duration, y0, y1, dyaw, rate,
               w_rad, w_deg, ratio,
               fl, rl, fr, rr, left, right, right - left)
        )
    if not segments:
        print("NO_MOTION_SEGMENT")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
