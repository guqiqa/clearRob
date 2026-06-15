#!/usr/bin/env python3
"""
YOLOv8 训练入口 — 基于 Ultralytics 库，所有权重参数可调。

基本用法:
  # 使用默认配置训练
  python scripts/train.py

  # 指定模型规模和数据集
  python scripts/train.py --model configs/model_yolov8s.yaml --data configs/dataset.yaml

  # 覆盖指定超参（可任意组合）
  python scripts/train.py \
      --model configs/model_yolov8n.yaml \
      --epochs 150 \
      --batch 32 \
      --lr0 0.0005 \
      --box 6.0 \
      --cls 0.8 \
      --mosaic 0.5 \
      --device 0

  # 断点续训
  python scripts/train.py --resume runs/train_xxx/weights/last.pt

可调参数（命令行覆盖 > YAML 配置）:
  --model     模型配置文件路径
  --data      数据集配置文件路径
  --epochs    训练轮数
  --batch     Batch size
  --imgsz     输入图像尺寸
  --device    GPU 编号 (0, 1, "cpu", "0,1")
  --lr0       初始学习率
  --lrf       最终学习率因子
  --optimizer 优化器 (AdamW/SGD/Adam)
  --box       边界框损失权重
  --cls       分类损失权重
  --dfl       DFL 损失权重
  --mosaic    Mosaic 增强概率
  --mixup     MixUp 增强概率
  --fliplr    水平翻转概率
  --scale     缩放增益
  --patience  早停 patience
  --cos_lr    余弦学习率调度
  --amp       自动混合精度
  --close_mosaic  最后 N epoch 关闭 mosaic
  --resume    断点续训的 checkpoint 路径
  --name      训练任务名称
  --exist_ok  覆盖同名输出目录
"""

import argparse
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


def build_train_args(cfg, cli_args):
    """将 YAML 配置 + CLI 参数合并为 ultralytics 训练参数字典"""
    m = cfg["model"]
    t = cfg["train"]
    o = cfg["optimizer"]
    l = cfg["loss"]
    a = cfg["augment"]
    s = cfg["strategy"]
    out = cfg["output"]

    # 命令行覆盖的优先级最高；用 getattr(cli_args, key, yaml_default) 模式
    def arg(key, default):
        val = getattr(cli_args, key, None)
        return default if val is None else val

    train_args = {
        # 数据
        "data": cli_args.data,
        # 模型
        "model": arg("pretrained", m.get("pretrained", "yolov8s.pt")),
        # 训练
        "epochs": arg("epochs", t.get("epochs", 300)),
        "batch": arg("batch", t.get("batch", 16)),
        "imgsz": arg("imgsz", t.get("imgsz", 640)),
        "device": str(arg("device", t.get("device", 0))),
        "workers": arg("workers", t.get("workers", 0)),  # Windows 推荐 0
        "resume": arg("resume", t.get("resume", False)),
        "exist_ok": arg("exist_ok", t.get("exist_ok", True)),
        # 优化器
        "optimizer": arg("optimizer", o.get("name", "AdamW")),
        "lr0": arg("lr0", o.get("lr0", 0.001)),
        "lrf": arg("lrf", o.get("lrf", 0.01)),
        "momentum": arg("momentum", o.get("momentum", 0.937)),
        "weight_decay": arg("weight_decay", o.get("weight_decay", 0.0005)),
        "warmup_epochs": arg("warmup_epochs", o.get("warmup_epochs", 3)),
        "warmup_momentum": arg("warmup_momentum", o.get("warmup_momentum", 0.8)),
        "warmup_bias_lr": arg("warmup_bias_lr", o.get("warmup_bias_lr", 0.1)),
        # 损失
        "box": arg("box", l.get("box", 7.5)),
        "cls": arg("cls", l.get("cls", 0.5)),
        "dfl": arg("dfl", l.get("dfl", 1.5)),
        # 增强
        "mosaic": arg("mosaic", a.get("mosaic", 1.0)),
        "mixup": arg("mixup", a.get("mixup", 0.1)),
        "copy_paste": arg("copy_paste", a.get("copy_paste", 0.1)),
        "hsv_h": arg("hsv_h", a.get("hsv_h", 0.015)),
        "hsv_s": arg("hsv_s", a.get("hsv_s", 0.7)),
        "hsv_v": arg("hsv_v", a.get("hsv_v", 0.4)),
        "degrees": arg("degrees", a.get("degrees", 0.0)),
        "translate": arg("translate", a.get("translate", 0.1)),
        "scale": arg("scale", a.get("scale", 0.5)),
        "fliplr": arg("fliplr", a.get("fliplr", 0.5)),
        "flipud": arg("flipud", a.get("flipud", 0.0)),
        "erasing": arg("erasing", a.get("erasing", 0.4)),
        # 策略
        "close_mosaic": arg("close_mosaic", s.get("close_mosaic", 10)),
        "label_smoothing": arg("label_smoothing", s.get("label_smoothing", 0.0)),
        "nbs": arg("nbs", s.get("nbs", 64)),
        "patience": arg("patience", s.get("patience", 50)),
        "cos_lr": arg("cos_lr", s.get("cos_lr", True)),
        "amp": arg("amp", s.get("amp", True)),
        "ema": arg("ema", s.get("ema", True)),
        "dropout": arg("dropout", s.get("dropout", 0.0)),
        "save_period": arg("save_period", s.get("save_period", 10)),
        # 输出
        "project": arg("project", out.get("project", "runs")),
        "name": arg("name", out.get("name", "train")),
        # 固定项
        "verbose": True,
        "seed": 42,
        "plots": True,
        "save": True,
    }

    # 如果 resume 是 True（CLI --resume 不带值），从 runs 找最新 last.pt
    if train_args["resume"] is True:
        train_args["resume"] = _find_latest_last_pt(train_args["project"])

    # name 自动加时间戳
    if train_args["name"] == "train":
        train_args["name"] = f"train_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    return train_args


def _find_latest_last_pt(project_dir):
    """在 project_dir 下找最新的 last.pt"""
    import glob
    candidates = sorted(
        glob.glob(os.path.join(project_dir, "**/last.pt"), recursive=True),
        key=os.path.getmtime,
        reverse=True,
    )
    return candidates[0] if candidates else False


def print_config(train_args):
    """打印当前生效的训练参数"""
    print("\n" + "=" * 55)
    print("  训练配置")
    print("=" * 55)
    groups = {
        "数据": ["data", "model", "imgsz"],
        "训练": ["epochs", "batch", "device", "workers", "resume"],
        "优化器": ["optimizer", "lr0", "lrf", "momentum", "weight_decay",
                 "warmup_epochs", "cos_lr", "amp"],
        "损失": ["box", "cls", "dfl"],
        "增强": ["mosaic", "mixup", "fliplr", "scale", "hsv_h", "hsv_s", "hsv_v",
                "degrees", "erasing", "close_mosaic"],
        "策略": ["patience", "ema", "dropout", "label_smoothing", "save_period"],
        "输出": ["project", "name"],
    }
    for group, keys in groups.items():
        print(f"\n  [{group}]")
        for k in keys:
            v = train_args.get(k, "—")
            print(f"    {k:<20} = {v}")


def main():
    parser = argparse.ArgumentParser(
        description="YOLOv8 训练 — 基于 Ultralytics，所有参数可调",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python scripts/train.py
  python scripts/train.py --model configs/model_yolov8s.yaml --epochs 200
  python scripts/train.py --lr0 0.0005 --box 6.0 --cls 0.8
  python scripts/train.py --resume runs/train_xxx/weights/last.pt
        """,
    )
    # 配置文件
    parser.add_argument("--model", default="configs/model_yolov8s.yaml", help="模型配置文件")
    parser.add_argument("--data", default="configs/dataset.yaml", help="数据集配置文件")
    # 训练超参（None = 使用 YAML 默认值）
    parser.add_argument("--epochs", type=int, default=None)
    parser.add_argument("--batch", type=int, default=None)
    parser.add_argument("--imgsz", type=int, default=None)
    parser.add_argument("--device", default=None)
    parser.add_argument("--workers", type=int, default=None)
    parser.add_argument("--resume", nargs="?", const=True, default=None, help="断点续训（不带值则自动找最新 last.pt）")
    parser.add_argument("--exist_ok", type=lambda x: x.lower() == "true", default=None)
    # 优化器
    parser.add_argument("--optimizer", default=None, choices=["AdamW", "SGD", "Adam", "RMSProp"])
    parser.add_argument("--lr0", type=float, default=None)
    parser.add_argument("--lrf", type=float, default=None)
    parser.add_argument("--momentum", type=float, default=None)
    parser.add_argument("--weight_decay", type=float, default=None)
    parser.add_argument("--warmup_epochs", type=int, default=None)
    parser.add_argument("--warmup_momentum", type=float, default=None)
    parser.add_argument("--warmup_bias_lr", type=float, default=None)
    # 损失
    parser.add_argument("--box", type=float, default=None)
    parser.add_argument("--cls", type=float, default=None)
    parser.add_argument("--dfl", type=float, default=None)
    # 增强
    parser.add_argument("--mosaic", type=float, default=None)
    parser.add_argument("--mixup", type=float, default=None)
    parser.add_argument("--copy_paste", type=float, default=None)
    parser.add_argument("--hsv_h", type=float, default=None)
    parser.add_argument("--hsv_s", type=float, default=None)
    parser.add_argument("--hsv_v", type=float, default=None)
    parser.add_argument("--degrees", type=float, default=None)
    parser.add_argument("--translate", type=float, default=None)
    parser.add_argument("--scale", type=float, default=None)
    parser.add_argument("--fliplr", type=float, default=None)
    parser.add_argument("--flipud", type=float, default=None)
    parser.add_argument("--erasing", type=float, default=None)
    # 策略
    parser.add_argument("--close_mosaic", type=int, default=None)
    parser.add_argument("--label_smoothing", type=float, default=None)
    parser.add_argument("--nbs", type=int, default=None)
    parser.add_argument("--patience", type=int, default=None)
    parser.add_argument("--cos_lr", type=lambda x: x.lower() == "true", default=None)
    parser.add_argument("--amp", type=lambda x: x.lower() == "true", default=None)
    parser.add_argument("--ema", type=lambda x: x.lower() == "true", default=None)
    parser.add_argument("--dropout", type=float, default=None)
    parser.add_argument("--save_period", type=int, default=None)
    # 输出
    parser.add_argument("--name", default=None)
    parser.add_argument("--project", default=None)
    # 其他
    parser.add_argument("--pretrained", default=None, help="预训练权重路径（覆盖 YAML 配置）")

    cli_args = parser.parse_args()

    # 转绝对路径
    if not os.path.isabs(cli_args.model):
        cli_args.model = str(PROJECT_ROOT / cli_args.model)
    if not os.path.isabs(cli_args.data):
        cli_args.data = str(PROJECT_ROOT / cli_args.data)

    # 加载 YAML 配置
    cfg = load_yaml(cli_args.model)

    # 构建 ultralytics 训练参数
    train_args = build_train_args(cfg, cli_args)

    # 打印配置
    print_config(train_args)

    # 训练
    print("\n" + "=" * 55)
    print("  开始训练")
    print("=" * 55 + "\n")

    from ultralytics import YOLO

    model = YOLO(train_args["model"])

    # ultralytics 接受的参数需要转为 kwargs
    ultralytics_keys = [
        "data", "epochs", "batch", "imgsz", "device", "workers", "resume",
        "exist_ok", "optimizer", "lr0", "lrf", "momentum", "weight_decay",
        "warmup_epochs", "warmup_momentum", "warmup_bias_lr",
        "box", "cls", "dfl", "mosaic", "mixup", "copy_paste",
        "hsv_h", "hsv_s", "hsv_v", "degrees", "translate", "scale",
        "fliplr", "flipud", "erasing", "close_mosaic", "label_smoothing",
        "nbs", "patience", "cos_lr", "amp", "ema", "dropout",
        "save_period", "project", "name", "verbose", "seed", "plots", "save",
    ]
    kwargs = {k: train_args[k] for k in ultralytics_keys if k in train_args}

    results = model.train(**kwargs)

    # 输出结果摘要
    print("\n" + "=" * 55)
    print("  训练完成")
    print("=" * 55)
    print(f"  最优模型: {results.save_dir}/weights/best.pt")
    print(f"  最后模型: {results.save_dir}/weights/last.pt")
    print(f"  日志目录: {results.save_dir}")


if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()
    main()
