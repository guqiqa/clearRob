#!/usr/bin/env python3
"""
YOLO V13 视频目标检测 — 实时预览 + 输出标注视频

用法:
    python detect_video.py <视频路径>
    python detect_video.py --webcam          # 使用摄像头

环境: conda activate yolo
"""

import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from ultralytics import YOLO

# ========== 配置 ==========
MODEL_PATH = Path(__file__).parent.parent / "模型" / "v13" / "best.pt"
OUTPUT_DIR = Path(__file__).parent / "output"
CONF_THRESH = 0.25

# 8 类名称（与 label_mapping_v13.yaml 一致）
CLASS_CN = ["可回收物", "厨余垃圾", "有害垃圾", "其他垃圾",
            "绿化垃圾", "行人",     "障碍物",   "污渍"]

# 每类 BGR 颜色
COLORS = [
    (0, 255, 0),     # 0 可回收物 — 绿
    (0, 165, 255),   # 1 厨余垃圾 — 橙
    (0, 0, 255),     # 2 有害垃圾 — 红
    (128, 128, 128), # 3 其他垃圾 — 灰
    (0, 128, 0),     # 4 绿化垃圾 — 深绿
    (255, 0, 0),     # 5 行人     — 蓝
    (0, 255, 255),   # 6 障碍物   — 黄
    (128, 0, 128),   # 7 污渍     — 紫
]

# ---- 中文字体 ----
def _load_font(size: int) -> ImageFont.FreeTypeFont:
    """按优先级查找可用的中文字体"""
    candidates = [
        "C:/Windows/Fonts/msyh.ttc",        # 微软雅黑
        "C:/Windows/Fonts/simhei.ttf",      # 黑体
        "C:/Windows/Fonts/simsun.ttc",      # 宋体
    ]
    for p in candidates:
        if Path(p).exists():
            return ImageFont.truetype(p, size)
    # 回退：PIL 默认字体（不支持中文，但不会崩溃）
    return ImageFont.load_default()


FONT = _load_font(18)


def draw_labels(img_bgr: np.ndarray, boxes) -> np.ndarray:
    """在 BGR 图像上用 PIL 绘制中文标签框"""
    # BGR → RGB → PIL
    img_pil = Image.fromarray(img_bgr[:, :, ::-1])
    draw = ImageDraw.Draw(img_pil)

    for box in boxes:
        xyxy = box.xyxy[0].cpu().numpy().astype(int)
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])

        label = CLASS_CN[cls_id]
        color_rgb = COLORS[cls_id][::-1]  # BGR → RGB
        text = f"{label} {conf:.2f}"

        # 文字尺寸
        bbox = draw.textbbox((0, 0), text, font=FONT)
        tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]

        # 标签背景 & 文字
        y_top = max(0, xyxy[1] - th - 4)
        draw.rectangle([xyxy[0], y_top, xyxy[0] + tw + 4, xyxy[1]], fill=color_rgb)
        draw.text((xyxy[0] + 2, y_top + 1), text, fill=(255, 255, 255), font=FONT)

        # 检测框
        draw.rectangle(xyxy.tolist(), outline=color_rgb, width=2)

    # PIL → BGR numpy
    return np.array(img_pil)[:, :, ::-1]


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--webcam")]
    use_webcam = "--webcam" in sys.argv

    # ---- 加载模型 ----
    if not MODEL_PATH.exists():
        sys.exit(f"[错误] 模型不存在: {MODEL_PATH}")
    print(f"[加载] {MODEL_PATH}")
    model = YOLO(str(MODEL_PATH))

    # ---- 确定视频源 ----
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    if use_webcam:
        source = 0
        out_name = "webcam_output.mp4"
    elif args:
        source = args[0]
        out_name = Path(source).stem + "_annotated.mp4"
    else:
        sys.exit("用法: python detect_video.py <视频路径> 或 --webcam")

    out_path = str(OUTPUT_DIR / out_name)

    # ---- 获取 FPS / 尺寸 ----
    cap = cv2.VideoCapture(source)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    w, h = int(cap.get(3)), int(cap.get(4))
    cap.release()

    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))

    # ---- 逐帧推理 ----
    print(f"[推理] 按 Q 退出 | 输出 → {out_path}")
    results = model(source, stream=True, conf=CONF_THRESH, verbose=False)

    for r in results:
        img = r.orig_img.copy()
        if r.boxes is not None:
            img = draw_labels(img, r.boxes)

        writer.write(img)
        cv2.imshow("YOLO V13", img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    writer.release()
    cv2.destroyAllWindows()
    print(f"[完成] {out_path}")


if __name__ == "__main__":
    main()
