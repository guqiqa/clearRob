"""
检测旋转/翻转产生的重复图片，每组只保留一张，其余移入 _duplicates/ 子文件夹。
用法: python dedup_rotated.py <目录> [--dry-run]
"""
import os, sys, argparse
from pathlib import Path
from collections import defaultdict

try:
    from PIL import Image
except ImportError:
    print("需要安装: pip install Pillow")
    sys.exit(1)

# ── 感知哈希 ──
def dhash(img, hash_size=8):
    """Difference hash，旋转不不变，但配合多变体可以检测旋转版本"""
    img = img.convert("L").resize((hash_size + 1, hash_size), Image.LANCZOS)
    pixels = list(img.getdata())
    h = 0
    for row in range(hash_size):
        for col in range(hash_size):
            h <<= 1
            idx = row * (hash_size + 1) + col
            h |= 1 if pixels[idx] > pixels[idx + 1] else 0
    return h

def hamming(a, b, bits=64):
    return bin(a ^ b).count("1")

def all_hashes(img):
    """返回图片的 8 个方向变体的 hash 集合 (0/90/180/270 × 翻转/不翻转)"""
    hashes = set()
    for rot in [0, 90, 180, 270]:
        for flip in [False, True]:
            v = img.rotate(rot, expand=True)
            if flip:
                v = v.transpose(Image.FLIP_LEFT_RIGHT)
            hashes.add(dhash(v))
    return hashes

# ── 主逻辑 ──
parser = argparse.ArgumentParser()
parser.add_argument("dir", help="图片目录")
parser.add_argument("--dry-run", action="store_true", help="仅预览")
args = parser.parse_args()

exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

images = []
for f in sorted(os.listdir(args.dir)):
    if Path(f).suffix.lower() in exts:
        images.append(os.path.join(args.dir, f))

print(f"扫描 {len(images)} 张图片")

if len(images) < 2:
    print("少于 2 张，无需去重"); sys.exit(0)

# 为每张图计算 8 向 hash
img_hashes = {}
for i, path in enumerate(images):
    try:
        img_hashes[path] = all_hashes(Image.open(path))
    except:
        pass
    if (i + 1) % 20 == 0:
        print(f"  hash: {i + 1}/{len(images)}")

print(f"hash 计算完成 ({len(img_hashes)} 张)")

# 分组：两张图有任何 hash 交集 → 同组
parent = {p: p for p in img_hashes}

def find(x):
    while parent[x] != x:
        parent[x] = parent[parent[x]]
        x = parent[x]
    return x

def union(a, b):
    ra, rb = find(a), find(b)
    if ra != rb:
        parent[ra] = rb

paths = list(img_hashes.keys())
for i in range(len(paths)):
    hi = img_hashes[paths[i]]
    for j in range(i + 1, len(paths)):
        if find(paths[i]) == find(paths[j]):
            continue
        if hi & img_hashes[paths[j]]:  # 有交集
            union(paths[i], paths[j])

groups = defaultdict(list)
for p in paths:
    groups[find(p)].append(p)

# 处理
to_move = []
kept = []
for g in groups.values():
    g.sort(key=lambda x: os.path.getsize(x), reverse=True)
    kept.append(g[0])
    to_move.extend(g[1:])

dup_dir = os.path.join(args.dir, "_duplicates")
if to_move:
    print(f"\n{len(kept)} 张保留, {len(to_move)} 张重复 → _duplicates/")
    for p in to_move:
        print(f"  {'[DRY]' if args.dry_run else '移动'}: {os.path.basename(p)}")
    if not args.dry_run:
        os.makedirs(dup_dir, exist_ok=True)
        for p in to_move:
            dst = os.path.join(dup_dir, os.path.basename(p))
            os.rename(p, dst)
            # 同名 JSON 也移走
            j = os.path.splitext(p)[0] + ".json"
            if os.path.exists(j):
                os.rename(j, os.path.join(dup_dir, os.path.basename(j)))
        print(f"完成，重复文件已移至 {dup_dir}")
else:
    print("未发现旋转/翻转重复图片。")
