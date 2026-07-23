#!/usr/bin/env python3
"""
cleaning_vision_v13 — 智能环卫机器人视觉检测 (V13 YOLO11s)

Usage:
  python detect.py --image path/to/photo.jpg           # 单张图片
  python detect.py --image path/to/photo.jpg --display  # 显示结果
  python detect.py --dir path/to/images/ --save          # 批量处理文件夹
  python detect.py --camera 0                            # USB摄像头
  python detect.py --video path/to/video.mp4 --save      # 视频文件

Output:
  --save      保存标注图片到 output/ 目录
  --json      输出 JSON 格式检测结果
  --display   弹窗显示检测结果（需要GUI）
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import List

import cv2
import numpy as np

# Add current dir to path so yolo_detector can be imported
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from yolo_detector import YOLODetector, Detection


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

CLASS_NAMES = [
    "recyclable",      # 0  可回收垃圾
    "kitchen_waste",   # 1  厨余垃圾
    "hazardous",       # 2  有害垃圾
    "other_waste",     # 3  其他垃圾
    "green_waste",     # 4  绿化垃圾
    "pedestrian",      # 5  行人/宠物
    "obstacle",        # 6  道路障碍物/车辆
    "stain",           # 7  污渍/积水
]

CLASS_LABELS_ZH = [
    "可回收", "厨余", "有害", "其他垃圾",
    "绿化垃圾", "行人/宠物", "障碍物", "污渍",
]

# BGR colors per class (for drawing)
COLORS = [
    (0, 255, 0),     # recyclable — green
    (0, 200, 255),   # kitchen_waste — yellow
    (0, 0, 255),     # hazardous — red
    (255, 200, 0),   # other_waste — cyan
    (0, 180, 100),   # green_waste — teal
    (255, 0, 255),   # pedestrian — magenta
    (0, 100, 255),   # obstacle — orange
    (128, 128, 128), # stain — gray
]

MODEL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model")
DEFAULT_MODEL = os.path.join(MODEL_DIR, "best.onnx")
DEFAULT_CONFIG = os.path.join(MODEL_DIR, "config.yaml")

# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------

def draw_detections(image: np.ndarray, detections: List[Detection]) -> np.ndarray:
    """Draw bounding boxes and labels on an image (returns annotated copy)."""
    canvas = image.copy()
    for d in detections:
        color = COLORS[d.class_id] if d.class_id < len(COLORS) else (255, 255, 255)
        zh = CLASS_LABELS_ZH[d.class_id] if d.class_id < len(CLASS_LABELS_ZH) else d.class_name
        label = f"{zh} {d.confidence:.2f}"

        cv2.rectangle(canvas, (d.xmin, d.ymin), (d.xmax, d.ymax), color, 2)

        # Label background
        (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(canvas, (d.xmin, d.ymin - th - 6), (d.xmin + tw + 4, d.ymin), color, -1)
        cv2.putText(canvas, label, (d.xmin + 2, d.ymin - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return canvas


def detections_to_dict(detections: List[Detection]) -> list:
    """Convert detections to JSON-serializable list."""
    return [
        {
            "class_id": d.class_id,
            "class_name": d.class_name,
            "confidence": round(d.confidence, 4),
            "bbox": {"xmin": d.xmin, "ymin": d.ymin, "xmax": d.xmax, "ymax": d.ymax},
        }
        for d in detections
    ]


# ---------------------------------------------------------------------------
# Inference helpers
# ---------------------------------------------------------------------------

def process_image(detector: YOLODetector, image_path: str, args) -> None:
    """Detect objects in a single image and handle output."""
    img = cv2.imread(image_path)
    if img is None:
        print(f"[ERROR] Cannot read image: {image_path}", file=sys.stderr)
        return

    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    detections, ms = detector.detect(img_rgb)

    print(f"\n  File: {image_path}")
    print(f"  Inference: {ms:.1f} ms,  {len(detections)} detections")
    for d in detections:
        zh = CLASS_LABELS_ZH[d.class_id] if d.class_id < len(CLASS_LABELS_ZH) else d.class_name
        print(f"    [{zh}] {d.class_name}  conf={d.confidence:.3f}  "
              f"box=({d.xmin},{d.ymin},{d.xmax},{d.ymax})  "
              f"size={d.xmax - d.xmin}x{d.ymax - d.ymin}")

    if args.json:
        print(json.dumps(detections_to_dict(detections), ensure_ascii=False, indent=2))

    if args.save:
        os.makedirs("output", exist_ok=True)
        out_name = os.path.splitext(os.path.basename(image_path))[0] + "_detected.jpg"
        out_path = os.path.join("output", out_name)
        annotated = draw_detections(img, detections)
        cv2.imwrite(out_path, annotated)
        print(f"  Saved: {out_path}")

    if args.display:
        annotated = draw_detections(img, detections)
        cv2.imshow("cleaning_vision_v13", annotated)
        cv2.waitKey(0)


def process_directory(detector: YOLODetector, dir_path: str, args) -> None:
    """Batch process all images in a directory."""
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"}
    files = sorted(
        f for f in Path(dir_path).iterdir()
        if f.suffix.lower() in exts
    )
    if not files:
        print(f"[ERROR] No image files found in: {dir_path}", file=sys.stderr)
        return

    print(f"Processing {len(files)} images...\n")

    total_ms = 0.0
    total_dets = 0
    for f in files:
        img = cv2.imread(str(f))
        if img is None:
            print(f"  [SKIP] {f.name}")
            continue
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        detections, ms = detector.detect(img_rgb)
        total_ms += ms
        total_dets += len(detections)

        status = f"{len(detections):2d} dets"
        print(f"  {f.name:40s}  {ms:6.1f} ms  {status}")

        if args.save:
            os.makedirs("output", exist_ok=True)
            annotated = draw_detections(img, detections)
            out_name = f.stem + "_detected.jpg"
            cv2.imwrite(os.path.join("output", out_name), annotated)

    n = len(files)
    print(f"\n  Done: {n} images, {total_dets} detections, avg {total_ms / n:.1f} ms/image")


def process_camera(detector: YOLODetector, cam_id: int, args) -> None:
    """Real-time detection from camera."""
    cap = cv2.VideoCapture(cam_id)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {cam_id}", file=sys.stderr)
        return

    print(f"Camera {cam_id} started. Press 'q' to quit, 's' to save screenshot.\n")

    frame_count = 0
    fps_start = time.perf_counter()
    fps_frames = 0
    current_fps = 0.0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            detections, ms = detector.detect(frame_rgb)
            frame_count += 1
            fps_frames += 1

            # FPS counter (update every second)
            now = time.perf_counter()
            elapsed = now - fps_start
            if elapsed >= 1.0:
                current_fps = fps_frames / elapsed
                fps_frames = 0
                fps_start = now

            annotated = draw_detections(frame, detections)

            # Overlay stats
            h, w = annotated.shape[:2]
            stats = f"FPS: {current_fps:.1f} | Inference: {ms:.1f}ms | Dets: {len(detections)}"
            cv2.putText(annotated, stats, (10, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.imshow("cleaning_vision_v13 — Camera", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                os.makedirs("output", exist_ok=True)
                fname = f"output/screenshot_{frame_count:04d}.jpg"
                cv2.imwrite(fname, annotated)
                print(f"  Screenshot saved: {fname}")
    finally:
        cap.release()
        cv2.destroyAllWindows()


def process_video(detector: YOLODetector, video_path: str, args) -> None:
    """Detect objects in a video file."""
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open video: {video_path}", file=sys.stderr)
        return

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps_video = cap.get(cv2.CAP_PROP_FPS)
    print(f"Video: {video_path}")
    print(f"Frames: {total_frames},  FPS: {fps_video:.1f}")

    writer = None
    if args.save:
        os.makedirs("output", exist_ok=True)
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        out_path = os.path.join("output", os.path.splitext(os.path.basename(video_path))[0] + "_detected.mp4")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(out_path, fourcc, fps_video, (w, h))
        print(f"Saving to: {out_path}")

    frame_idx = 0
    total_ms = 0.0
    total_dets = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            detections, ms = detector.detect(frame_rgb)
            total_ms += ms
            total_dets += len(detections)
            frame_idx += 1

            if frame_idx % 30 == 0:
                print(f"  Frame {frame_idx}/{total_frames}  "
                      f"avg {total_ms / frame_idx:.1f} ms  "
                      f"dets: {len(detections)}")

            if args.save and writer is not None:
                annotated = draw_detections(frame, detections)
                writer.write(annotated)

            if args.display:
                annotated = draw_detections(frame, detections)
                cv2.imshow("cleaning_vision_v13 — Video", annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()

    n = max(frame_idx, 1)
    print(f"\n  Done: {frame_idx} frames, {total_dets} detections, avg {total_ms / n:.1f} ms/frame")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="cleaning_vision_v13 — YOLO11s 8-class garbage/obstacle detector",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python detect.py --image photo.jpg
  python detect.py --image photo.jpg --display --json
  python detect.py --dir ./test_images/ --save
  python detect.py --camera 0
  python detect.py --video road.mp4 --save
        """,
    )
    # Input mode (mutually exclusive)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--image", "-i", help="Single image file path")
    group.add_argument("--dir", "-d", help="Directory of images to batch process")
    group.add_argument("--camera", "-c", type=int, help="Camera device index (e.g. 0)")
    group.add_argument("--video", "-v", help="Video file path")

    # Output options
    parser.add_argument("--save", "-s", action="store_true", help="Save annotated images/video to output/")
    parser.add_argument("--json", "-j", action="store_true", help="Output detections as JSON")
    parser.add_argument("--display", "-p", action="store_true", help="Show detection window (requires GUI)")

    # Model options
    parser.add_argument("--model", "-m", default=DEFAULT_MODEL, help=f"ONNX model path (default: model/best.onnx)")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold (default: 0.25)")
    parser.add_argument("--iou", type=float, default=0.45, help="NMS IoU threshold (default: 0.45)")

    args = parser.parse_args()

    # Validate model
    if not os.path.exists(args.model):
        print(f"[ERROR] Model not found: {args.model}", file=sys.stderr)
        print("  Place best.onnx in the model/ directory or use --model PATH", file=sys.stderr)
        sys.exit(1)

    # Load detector
    print(f"Model:  {args.model}")
    print(f"Conf:   {args.conf}")
    print(f"IoU:    {args.iou}")
    print("Loading...", end=" ", flush=True)

    detector = YOLODetector(
        model_path=args.model,
        class_names=CLASS_NAMES,
        conf_threshold=args.conf,
        iou_threshold=args.iou,
    )
    if not detector.load():
        print("\n[ERROR] Failed to load model. Check onnxruntime installation.", file=sys.stderr)
        sys.exit(1)

    print(f"OK (classes={len(CLASS_NAMES)})")

    # Run
    t0 = time.perf_counter()
    if args.image:
        process_image(detector, args.image, args)
    elif args.dir:
        process_directory(detector, args.dir, args)
    elif args.camera is not None:
        process_camera(detector, args.camera, args)
    elif args.video:
        process_video(detector, args.video, args)

    elapsed = time.perf_counter() - t0
    print(f"\nTotal time: {elapsed:.1f}s")


if __name__ == "__main__":
    main()
