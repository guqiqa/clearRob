#!/usr/bin/env python3
"""
多模型对比 — 在同一数据集上横向对比多个模型的精度与速度。

用法:
  python scripts/compare.py \
      --models v1:../训练程序/runs/train_v1/weights/best.pt \
               v2:../训练程序/runs/train_v2/weights/best.pt \
               baseline:baseline/best.pt \
      --data ../../训练程序/configs/dataset.yaml
"""

import argparse
import os
import sys
from collections import OrderedDict
from pathlib import Path

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


def main():
    parser = argparse.ArgumentParser(description="多模型对比评估")
    parser.add_argument("--models", nargs="+", required=True,
                        help="模型列表，格式: label:path (如 v1:best.pt)")
    parser.add_argument("--data", default="../../训练程序/configs/dataset.yaml", help="数据集配置")
    parser.add_argument("--device", default="0", help="设备")
    parser.add_argument("--output", default=None, help="输出目录")

    args = parser.parse_args()

    data_yaml = str(PROJECT_ROOT / args.data) if not os.path.isabs(args.data) else args.data
    output_dir = Path(args.output) if args.output else PROJECT_ROOT / "reports" / "compare"
    output_dir.mkdir(parents=True, exist_ok=True)

    # 解析模型列表
    models = OrderedDict()
    for item in args.models:
        if ":" in item:
            label, path = item.split(":", 1)
        else:
            label = os.path.basename(item)
            path = item
        models[label] = path

    from ultralytics import YOLO

    results = OrderedDict()

    print("=" * 55)
    print("  多模型对比")
    print("=" * 55)

    for label, path in models.items():
        print(f"\n[{label}] {path}")
        model = YOLO(path)
        metrics = model.val(data=data_yaml, device=args.device, workers=0, verbose=False)

        box = metrics.box
        results[label] = {
            "path": path,
            "map50": round(float(box.map50), 4),
            "map50_95": round(float(box.map75), 4),
            "precision": round(float(box.mp), 4),
            "recall": round(float(box.mr), 4),
            "f1": round(2 * float(box.mp) * float(box.mr) /
                        (float(box.mp) + float(box.mr) + 1e-8), 4),
        }
        print(f"  mAP@0.5={results[label]['map50']}, Recall={results[label]['recall']}")

    # 输出对比表
    header = f"{'模型':<15} {'mAP@0.5':>10} {'mAP@.5:.95':>10} {'Precision':>10} {'Recall':>10} {'F1':>10}"
    separator = "-" * len(header)

    print("\n" + separator)
    print(header)
    print(separator)
    best_idx = max(results, key=lambda k: results[k]["map50"])
    for label, r in results.items():
        marker = " ← best" if label == best_idx else ""
        print(f"{label:<15} {r['map50']:>10.4f} {r['map50_95']:>10.4f} "
              f"{r['precision']:>10.4f} {r['recall']:>10.4f} {r['f1']:>10.4f}{marker}")
    print(separator)

    # 保存 JSON
    import json
    compare_path = output_dir / "compare.json"
    with open(compare_path, "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2)
    print(f"\n对比结果: {compare_path}")


if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()
    main()
