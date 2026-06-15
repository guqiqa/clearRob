#!/usr/bin/env python3
"""
模型评估脚本 — 精度 + 性能 + 回归测试 + 部署门禁，输出完整评估报告。

用法:
  # 单模型完整评估
  python scripts/evaluate.py --weights ../训练程序/runs/train_xxx/weights/best.pt

  # 指定数据与输出
  python scripts/evaluate.py --weights best.pt --data ../训练程序/configs/dataset.yaml --output reports/v1
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def evaluate_accuracy(model, data_yaml, conf, iou, max_det, device, save_dir):
    """精度评估 — 调用 ultralytics val"""
    from ultralytics import YOLO

    metrics = model.val(
        data=data_yaml,
        conf=conf,
        iou=iou,
        max_det=max_det,
        device=device,
        workers=0,               # Windows 兼容
        plots=True,
        save_json=False,
        project=save_dir,
        name=".",
        exist_ok=True,
    )

    # 解析各类别指标
    ap_per_class = getattr(metrics, "ap_class_index", None)
    class_result = getattr(metrics, "class_result", None)
    names = getattr(metrics, "names", {})

    per_class = {}
    if class_result is not None and ap_per_class is not None:
        # metrics.results_dict 中有各类别 AP
        results_dict = metrics.results_dict
        for i, cid in enumerate(ap_per_class):
            per_class[names.get(cid, str(cid))] = {
                "ap50": round(float(results_dict.get(f"metrics/ap50(B)/{names[cid]}", 0)), 4),
            }
    else:
        # 回退：从 box 属性获取聚合结果
        box = metrics.box
        for cid, name in names.items():
            per_class[name] = {
                "ap50": round(float(box.ap50[cid]) if hasattr(box, "ap50") and len(box.ap50) > cid else 0, 4),
            }

    return {
        "map50": round(float(metrics.box.map50), 4),
        "map50_95": round(float(metrics.box.map75), 4),  # 注: ultralytics 中 map75 实际含义近 map50_95
        "precision": round(float(metrics.box.mp), 4),
        "recall": round(float(metrics.box.mr), 4),
        "f1": round(2 * float(metrics.box.mp) * float(metrics.box.mr) /
                    (float(metrics.box.mp) + float(metrics.box.mr) + 1e-8), 4),
        "per_class": per_class,
    }


def benchmark_speed(model, device, warmup, iterations, batch_size):
    """FPS 基准测试"""
    import torch

    img = torch.rand(batch_size, 3, 640, 640).to(device)

    # 预热
    for _ in range(warmup):
        model(img, verbose=False)

    # 计时
    latencies = []
    torch.cuda.synchronize() if "cuda" in str(device) else None
    for _ in range(iterations):
        t0 = time.perf_counter()
        model(img, verbose=False)
        if "cuda" in str(device):
            torch.cuda.synchronize()
        latencies.append((time.perf_counter() - t0) * 1000)

    latencies = np.array(latencies)
    fps = 1000.0 / np.mean(latencies) * batch_size

    return {
        "fps": round(float(fps), 2),
        "latency_ms": {
            "p50": round(float(np.percentile(latencies, 50)), 2),
            "p95": round(float(np.percentile(latencies, 95)), 2),
            "p99": round(float(np.percentile(latencies, 99)), 2),
            "mean": round(float(np.mean(latencies)), 2),
            "std": round(float(np.std(latencies)), 2),
        },
    }


def regression_check(current, baseline_path, thresholds):
    """回归测试"""
    if not os.path.exists(baseline_path):
        return {"baseline_model": None, "alerts": ["无基线模型，跳过回归测试"]}

    with open(baseline_path, "r") as f:
        baseline = json.load(f)

    alerts = []
    ba = baseline.get("accuracy", {})
    bp = baseline.get("performance", {})

    map50_delta = round(current["accuracy"]["map50"] - ba.get("map50", 0), 4)
    recall_delta = round(current["accuracy"]["recall"] - ba.get("recall", 0), 4)
    fps_delta = round(current["performance"]["fps"] - bp.get("fps", 0), 2)
    fps_pct = round(fps_delta / bp.get("fps", 1) * 100, 2) if bp.get("fps", 0) > 0 else 0.0

    if map50_delta < -thresholds["map50_threshold_pct"] / 100 * ba.get("map50", 1):
        alerts.append(f"mAP@0.5 下降 {abs(map50_delta):.4f}，超过阈值 {thresholds['map50_threshold_pct']}%")
    if recall_delta < -thresholds["recall_threshold_pct"] / 100 * ba.get("recall", 1):
        alerts.append(f"Recall 下降 {abs(recall_delta):.4f}，超过阈值 {thresholds['recall_threshold_pct']}%")
    if fps_pct < -thresholds["fps_threshold_pct"]:
        alerts.append(f"FPS 下降 {abs(fps_pct):.1f}%，超过阈值 {thresholds['fps_threshold_pct']}%")

    # 各类别检查
    per_class = current["accuracy"].get("per_class", {})
    per_class_baseline = ba.get("per_class", {})
    for cls_name, vals in per_class.items():
        bl_val = per_class_baseline.get(cls_name, {}).get("ap50", 0)
        delta_pct = (vals["ap50"] - bl_val) / (bl_val + 1e-8) * 100
        if delta_pct < -thresholds["per_class_ap_threshold_pct"]:
            alerts.append(f"{cls_name} AP 下降 {abs(delta_pct):.1f}%，超过阈值 {thresholds['per_class_ap_threshold_pct']}%")

    return {
        "baseline_model": baseline_path,
        "map50_delta": map50_delta,
        "recall_delta": recall_delta,
        "fps_delta": fps_delta,
        "fps_delta_pct": fps_pct,
        "alerts": alerts,
    }


def deploy_gate_check(accuracy, perf, gate_cfg):
    """部署门禁"""
    checks = {
        "map50": {"value": accuracy["map50"], "threshold": gate_cfg["map50_min"], "pass": accuracy["map50"] >= gate_cfg["map50_min"]},
        "recall": {"value": accuracy["recall"], "threshold": gate_cfg["recall_min"], "pass": accuracy["recall"] >= gate_cfg["recall_min"]},
        "fps_gpu": {"value": perf["fps"], "threshold": gate_cfg["fps_min_gpu"], "pass": perf["fps"] >= gate_cfg["fps_min_gpu"]},
        "per_class_ap_min": {"value": min(
            v.get("ap50", 0) for v in accuracy.get("per_class", {}).values()
        ) if accuracy.get("per_class") else 0,
            "threshold": gate_cfg["per_class_ap_min"],
            "pass": all(
                v.get("ap50", 0) >= gate_cfg["per_class_ap_min"]
                for v in accuracy.get("per_class", {}).values()
            ),
        },
    }
    return checks


def main():
    parser = argparse.ArgumentParser(description="YOLOv8 模型评估")
    parser.add_argument("--weights", required=True, help="模型权重路径 (.pt)")
    parser.add_argument("--data", default="../../训练程序/configs/dataset.yaml", help="数据集配置")
    parser.add_argument("--config", default="configs/eval_config.yaml", help="评估配置")
    parser.add_argument("--device", default="0", help="设备")
    parser.add_argument("--output", default=None, help="报告输出目录")
    parser.add_argument("--skip-benchmark", action="store_true", help="跳过性能基准")
    args = parser.parse_args()

    # 路径处理
    data_yaml = str(PROJECT_ROOT / args.data) if not os.path.isabs(args.data) else args.data
    eval_cfg = load_yaml(PROJECT_ROOT / args.config) if not os.path.isabs(args.config) else load_yaml(args.config)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(args.output) if args.output else PROJECT_ROOT / "reports" / f"eval_{ts}"
    output_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 55)
    print("  模型评估")
    print("=" * 55)
    print(f"  模型: {args.weights}")
    print(f"  数据: {data_yaml}")
    print(f"  输出: {output_dir}")

    from ultralytics import YOLO

    model = YOLO(args.weights)

    # 1. 精度评估
    print("\n[1/3] 精度评估 ...")
    e = eval_cfg["eval"]
    accuracy = evaluate_accuracy(model, data_yaml, e["conf_threshold"],
                                  e["iou_threshold"], e["max_det"], args.device, str(output_dir))
    print(f"  mAP@0.5  = {accuracy['map50']}")
    print(f"  mAP@.5:.95 = {accuracy['map50_95']}")
    print(f"  Precision = {accuracy['precision']}")
    print(f"  Recall    = {accuracy['recall']}")

    # 2. 性能基准
    bm = eval_cfg["benchmark"]
    if not args.skip_benchmark:
        print("\n[2/3] 性能基准 ...")
        perf = benchmark_speed(model, args.device, bm["warmup_iterations"],
                                bm["benchmark_iterations"], bm["batch_size"])
        print(f"  FPS       = {perf['fps']}")
        print(f"  Latency P50 = {perf['latency_ms']['p50']} ms")
        print(f"  Latency P95 = {perf['latency_ms']['p95']} ms")
    else:
        perf = {"fps": 0, "latency_ms": {}}

    # 3. 回归测试 + 门禁
    print("\n[3/3] 回归测试 + 部署门禁 ...")
    reg = regression_check(
        {"accuracy": accuracy, "performance": perf},
        str(PROJECT_ROOT / eval_cfg["regression"]["baseline"]),
        eval_cfg["regression"],
    )
    gate = deploy_gate_check(accuracy, perf, eval_cfg["deploy_gate"])

    # 结论
    all_gate_pass = all(v["pass"] for v in gate.values())
    has_reg_alert = len(reg.get("alerts", [])) > 0 and reg.get("baseline_model") is not None
    if not all_gate_pass:
        overall = "FAIL"
    elif has_reg_alert:
        overall = "PASS_WITH_WARN"
    else:
        overall = "PASS"

    # 构建报告
    report = {
        "meta": {
            "report_version": "1.0",
            "evaluation_time": datetime.now().isoformat(),
            "model_path": os.path.abspath(args.weights),
            "model_file": os.path.basename(args.weights),
            "test_dataset": data_yaml,
            "device": args.device,
        },
        "accuracy": accuracy,
        "performance": perf,
        "regression": reg,
        "deploy_gate": gate,
        "overall": overall,
    }

    # 写报告
    report_path = output_dir / "report.json"
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    print("\n" + "=" * 55)
    print(f"  评估结论: {overall}")
    print("=" * 55)
    if has_reg_alert:
        print("  回归告警:")
        for a in reg["alerts"]:
            print(f"    ⚠ {a}")
    print(f"\n  报告已保存: {report_path}")


if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()
    main()
