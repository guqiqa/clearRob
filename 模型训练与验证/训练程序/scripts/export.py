#!/usr/bin/env python3
"""
模型导出脚本 — 将训练产出的 .pt 转换为 ONNX / TensorRT 格式。

用法:
  python scripts/export.py --weights runs/train_xxx/weights/best.pt
  python scripts/export.py --weights best.pt --format onnx tensorrt --imgsz 640 --half
"""

import argparse
import sys
from pathlib import Path

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description="YOLOv8 模型导出 (ONNX / TensorRT)")
    parser.add_argument("--weights", required=True, help="源 .pt 权重路径")
    parser.add_argument("--format", nargs="+", default=["onnx"], choices=["onnx", "engine", "openvino", "tflite"],
                        help="目标格式（可多个）")
    parser.add_argument("--imgsz", type=int, default=640, help="导出图像尺寸")
    parser.add_argument("--half", action="store_true", help="FP16 半精度")
    parser.add_argument("--int8", action="store_true", help="INT8 量化（仅 TensorRT）")
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset 版本")
    parser.add_argument("--simplify", action="store_true", default=True, help="ONNX 模型简化")
    parser.add_argument("--workspace", type=float, default=4.0, help="TensorRT workspace (GB)")
    parser.add_argument("--device", default="0", help="导出设备")
    parser.add_argument("--output", default=None, help="输出目录（默认与源文件同目录）")

    args = parser.parse_args()

    weights_path = Path(args.weights)
    if not weights_path.exists():
        print(f"错误: 权重文件不存在 — {args.weights}")
        sys.exit(1)

    model = YOLO(str(weights_path))

    for fmt in args.format:
        print(f"\n导出 → {fmt.upper()} ...")
        export_kwargs = {
            "format": fmt,
            "imgsz": args.imgsz,
            "half": args.half,
            "device": args.device,
        }
        if fmt == "onnx":
            export_kwargs["opset"] = args.opset
            export_kwargs["simplify"] = args.simplify
        if fmt == "engine":
            export_kwargs["workspace"] = args.workspace
            if args.int8:
                export_kwargs["int8"] = True

        exported_path = model.export(**export_kwargs)
        print(f"  输出: {exported_path}")


if __name__ == "__main__":
    main()
