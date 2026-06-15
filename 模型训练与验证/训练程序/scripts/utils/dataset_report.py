import os, sys
from collections import defaultdict, Counter
from pathlib import Path

PROCESSED = r"d:\AIview\模型训练与验证\数据集\processed"

# Source prefixes in filenames tell us which dataset each image came from
PREFIXES = {
    "taco_": "TACO (COCO检测)",
    "gc_": "garbage_classify (分类→检测)",
    "street_obstacle_": "street_obstacle (YOLO重映射)",
    "tn_": "trashnet (文件夹→检测)",
    "leaf_": "leaf_classify (树叶→green_waste)",
    "dat_": "datas (COCO检测, 行人为主)",
    "pdl_": "puddle (VOC积水)",
    "puddle_large_": "puddle_large (YOLO积水)",
}

CLASS_NAMES = {
    0: "recyclable",
    1: "kitchen_waste",
    2: "hazardous",
    3: "other_waste",
    4: "green_waste",
    5: "large_obstacle",
    6: "pedestrian_pet",
    7: "stain",
}

# Per-source stats
source_imgs = Counter()
source_boxes = Counter()
source_class_boxes = defaultdict(lambda: Counter())

# Global class distribution
class_total = Counter()
total_imgs = 0
total_boxes = 0

for split in ["train", "val", "test"]:
    lbl_dir = Path(PROCESSED) / split / "labels"
    if not lbl_dir.exists():
        continue
    for lbl_file in lbl_dir.iterdir():
        if not lbl_file.suffix == ".txt":
            continue

        # Determine source from filename
        fname = os.path.splitext(lbl_file.name)[0]
        source = "unknown"
        for prefix, label in PREFIXES.items():
            if fname.startswith(prefix):
                source = label
                break

        source_imgs[source] += 1
        total_imgs += 1

        with open(lbl_file) as f:
            for line in f:
                parts = line.strip().split()
                if not parts:
                    continue
                cid = int(parts[0])
                class_total[cid] += 1
                source_boxes[source] += 1
                source_class_boxes[source][cid] += 1
                total_boxes += 1

# Output
print("=" * 80)
print("  MERGED DATASET COMPOSITION")
print("=" * 80)

print(f"\n  Total images: {total_imgs:,}")
print(f"  Total boxes:  {total_boxes:,}")
print(f"  Splits: train/val/test")

print(f"\n{'─' * 80}")
print(f"  {'Source':<35} {'Images':>8} {'Boxes':>9} {'%Imgs':>7} {'%Boxes':>7}")
print(f"  {'─' * 35} {'─' * 8} {'─' * 9} {'─' * 7} {'─' * 7}")

for src_name, img_cnt in source_imgs.most_common():
    box_cnt = source_boxes[src_name]
    img_pct = img_cnt / total_imgs * 100
    box_pct = box_cnt / total_boxes * 100
    print(f"  {src_name:<35} {img_cnt:>8,} {box_cnt:>9,} {img_pct:>6.1f}% {box_pct:>6.1f}%")

# Per-source class breakdown
print(f"\n{'─' * 80}")
print(f"  PER-SOURCE CLASS DISTRIBUTION")
print(f"  {'─' * 80}")

header = f"  {'Source':<35} " + " ".join(f"{CLASS_NAMES[c]:>10}" for c in range(8))
print(header)
print(f"  {'─' * 35} " + " ".join(f"{'─' * 10}" for _ in range(8)))

for src_name, img_cnt in source_imgs.most_common():
    vals = [source_class_boxes[src_name].get(c, 0) for c in range(8)]
    row = f"  {src_name:<35} " + " ".join(f"{v:>10,}" for v in vals)
    print(row)

# Summary: class per-source contribution
print(f"\n{'─' * 80}")
print(f"  CLASS TOTALS BY SOURCE")
print(f"  {'─' * 80}")

for cid in range(8):
    name = CLASS_NAMES[cid]
    total = class_total[cid]
    print(f"\n  [{name}] total={total:,}")
    for src_name, _ in source_imgs.most_common():
        cnt = source_class_boxes[src_name].get(cid, 0)
        if cnt > 0:
            pct = cnt / total * 100 if total else 0
            bar = "=" * int(pct / 2)
            print(f"    {src_name:<35} {cnt:>7,}  ({pct:>5.1f}%) {bar}")

print(f"\n{'─' * 80}")
print(f"  IMBALANCE RATIO: max/min = {max(class_total.values()) / min(class_total.values()):.1f}:1")
print(f"  person dominates at {class_total[6]} boxes ({class_total[6]/total_boxes*100:.1f}%)")
print(f"{'─' * 80}")
