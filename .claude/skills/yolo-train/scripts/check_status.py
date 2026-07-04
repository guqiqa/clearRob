#!/usr/bin/env python3
"""Quick training status check — no imports beyond stdlib."""
import csv, os, sys, time
from pathlib import Path

def check(results_csv):
    if not os.path.exists(results_csv):
        print("STATUS: No results yet (Epoch 1 still running)")
        return

    with open(results_csv) as f:
        rows = list(csv.DictReader(f))

    n = len(rows)
    m50 = [float(r["metrics/mAP50(B)"]) for r in rows]
    peak = max(m50)
    peak_ep = m50.index(peak) + 1
    latest = m50[-1]
    tm = [float(r["time"]) for r in rows]

    mtime = os.path.getmtime(results_csv)
    age_min = (time.time() - mtime) / 60

    print(f"Epochs: {n} | {age_min:.0f}min ago | Peak: {peak:.4f}@E{peak_ep} | Latest: {latest:.4f}@E{n}")
    print(f"Elapsed: {tm[-1]/3600:.1f}h | ~{(tm[-1]-tm[-2])/60:.1f}min/ep" if n >= 2 else "")

    # Health checks
    collapsed = any(x < 0.01 for x in m50[-5:])
    if collapsed:
        print("CRASH DETECTED — mAP50 near zero")
        # Find crash point
        for i in range(peak_ep, n):
            if m50[i] < 0.01:
                print(f"  Crashed at E{i+1} (peak was E{peak_ep})")
                break
        return

    if n >= 20:
        # val_cls rising check
        vc_recent = []
        for i in range(max(0, n-10), n):
            vc = rows[i].get("val/cls_loss", "inf")
            try: vc = float(vc)
            except: vc = 999
            vc_recent.append(vc)
        if len(vc_recent) >= 5 and vc_recent[-1] > vc_recent[0] * 1.5 and vc_recent[-1] > 1.5:
            print(f"WARNING: val_cls rising — possible overfitting ({vc_recent[0]:.2f} → {vc_recent[-1]:.2f})")

    # Trend
    if n >= 5:
        t5 = m50[-1] - m50[-5]
        status = "climbing" if t5 > 0.003 else ("declining!" if t5 < -0.01 else "plateau")
        print(f"5ep trend: {t5:+.4f} ({status})")

    # Key milestones
    print("\nKey epochs:")
    for ep in [1, 3, 5, 10, 15, 20, 30, 50, 100]:
        if ep <= n:
            r = rows[ep-1]
            marker = " *BEST" if ep == peak_ep else ""
            print(f"  E{ep:<4} mAP50={float(r['metrics/mAP50(B)']):.4f} R={float(r['metrics/recall(B)']):.4f} P={float(r['metrics/precision(B)']):.4f}{marker}")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        # Try to find results.csv from cwd
        path = "runs/*/results.csv"
        matches = list(Path(".").glob(path))
        if not matches:
            print("Usage: python check_status.py <path/to/results.csv>")
            sys.exit(1)
        path = str(matches[0])
    check(path)
