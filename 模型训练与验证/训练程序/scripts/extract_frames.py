"""
从视频中均匀抽帧，保存为图片数据集。
用法: python extract_frames.py <视频路径> <输出目录> [--interval N] [--max N]
"""
import cv2
import os
import argparse

parser = argparse.ArgumentParser(description="从视频中均匀抽取帧")
parser.add_argument("video", help="视频文件路径")
parser.add_argument("output_dir", help="输出目录")
parser.add_argument("--interval", type=int, default=30,
                    help="每隔多少帧取一张（默认30，约1秒1张/30fps）")
parser.add_argument("--max", dest="max_frames", type=int, default=0,
                    help="最多取多少张（0=不限）")
args = parser.parse_args()

os.makedirs(args.output_dir, exist_ok=True)

cap = cv2.VideoCapture(args.video)
if not cap.isOpened():
    print(f"无法打开视频: {args.video}")
    exit(1)

total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
fps = cap.get(cv2.CAP_PROP_FPS)
duration = total_frames / fps if fps > 0 else 0
print(f"视频信息: {total_frames} 帧, {fps:.1f} fps, {duration:.1f} 秒")

saved = 0
frame_idx = 0
skip_until = 2 * fps  # 跳过前2秒（通常是无意义画面）

while True:
    ret, frame = cap.read()
    if not ret:
        break

    if frame_idx >= skip_until and frame_idx % args.interval == 0:
        name = f"vid_{saved + 1:04d}.jpg"
        cv2.imwrite(os.path.join(args.output_dir, name), frame)
        saved += 1
        if args.max_frames and saved >= args.max_frames:
            break

    frame_idx += 1

cap.release()
print(f"完成: 从 {frame_idx} 帧中抽取了 {saved} 张，保存到 {args.output_dir}")
