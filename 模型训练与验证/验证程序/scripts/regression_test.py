#!/usr/bin/env python3
"""
回归测试 — 新模型 vs 基线全指标对比，CI 友好（不合格返回非零退出码）。

用法:
  python scripts/regression_test.py --weights best.pt --baseline baseline/baseline.json

CI 集成:
  python scripts/regression_test.py --weights best.pt --ci   # exit code != 0 on fail
"""

import argparse
import json
import os
import sys
from datetime import datetime
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(description="回归测试")
    parser.add_argument("--weights", required=True, help="待测模型")
    parser.add_argument("--data", default="../../训练程序/configs/dataset.yaml")
    parser.add_argument("--config", default="configs/eval_config.yaml")
    parser.add_argument("--baseline", default="baseline/baseline.json")
    parser.add_argument("--device", default="0")
    parser.add_argument("--ci", action="store_true", help="CI 模式（不通过返回 exit code 1）")
    args = parser.parse_args()

    data_yaml = str(PROJECT_ROOT / args.data) if not os.path.isabs(args.data) else args.data
    baseline_path = str(PROJECT_ROOT / args.baseline) if not os.path.isabs(args.baseline) else args.baseline
    eval_cfg = load_yaml(PROJECT_ROOT / args.config) if not os.path.isabs(args.config) else load_yaml(args.config)

    print("=" * 55)
    print("  回归测试")
    print("=" * 55)

    # 快速评估
    from ultralytics import YOLO

    model = YOLO(args.weights)
    metrics = model.val(data=data_yaml, device=args.device, workers=0, verbose=False)
    box = metrics.box

    current = {
        "map50": round(float(box.map50), 4),
        "map50_95": round(float(box.map75), 4),
        "precision": round(float(box.mp), 4),
        "recall": round(float(box.mr), 4),
        "f1": round(2 * float(box.mp) * float(box.mr) /
                    (float(box.mp) + float(box.mr) + 1e-8), 4),
    }

    print(f"  mAP@0.5  = {current['map50']}")
    print(f"  mAP@.5:.95 = {current['map50_95']}")
    print(f"  Precision = {current['precision']}")
    print(f"  Recall    = {current['recall']}")

    # 加载基线
    if not os.path.exists(baseline_path):
        print(f"\n  [信息] 基线文件不存在: {baseline_path}")
        print(f"  将当前模型指标保存为基线")
        os.makedirs(os.path.dirname(baseline_path), exist_ok=True)
        with open(baseline_path, "w") as f:
            json.dump(current, f, indent=2)
        print(f"  基线已保存: {baseline_path}")
        sys.exit(0)

    with open(baseline_path, "r") as f:
        baseline = json.load(f)

    # 对比
    thresholds = eval_cfg["regression"]
    alerts = []

    checks = [
        ("mAP@0.5", "map50", thresholds["map50_threshold_pct"], 0.01),
        ("Recall", "recall", thresholds["recall_threshold_pct"], 0.01),
    ]

    for name, key, thresh_pct, abs_min in checks:
        bl = baseline.get(key, 0)
        cur = current[key]
        delta = cur - bl
        delta_pct = delta / (bl + 1e-8) * 100
        status = "OK"
        if delta_pct < -thresh_pct and abs(delta) > abs_min:
            alerts.append(f"{name}: {bl:.4f} → {cur:.4f} ({delta_pct:+.1f}%)  [超过阈值 {thresh_pct}%]")
            status = "FAIL"
        print(f"  {name:<15} {bl:.4f} → {cur:.4f}  ({delta_pct:+.1f}%)  [{status}]")

    # 结论
    if alerts:
        print(f"\n  ❌ 回归测试失败 ({len(alerts)} 项不通过):")
        for a in alerts:
            print(f"     {a}")
        sys.exit(1 if args.ci else 0)
    else:
        print(f"\n  ✅ 回归测试通过")
        # 更新基线
        with open(baseline_path, "w") as f:
            json.dump({
                **current,
                "_updated": datetime.now().isoformat(),
                "_previous": baseline,
            }, f, indent=2)
        print(f"  基线已更新: {baseline_path}")


if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()
    main()
