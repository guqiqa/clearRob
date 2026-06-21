#!/usr/bin/env python3
"""
数据集准备脚本 — 公开数据集的标签映射 + 格式转换 + 数据划分。

支持的源格式:
  - coco_taco:        TACO COCO JSON（检测框）
  - classify_csv:     分类标注 (filename.jpg, class_id) → 整图检测框
  - yolo_txt:         已是标准 YOLO 格式，直接拷贝

用法:
  python scripts/prepare_dataset.py                     # 默认: 处理全部
  python scripts/prepare_dataset.py --skip taco         # 跳过指定源
"""

import argparse
import json
import os
import random
import shutil
import sys
from collections import defaultdict
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent

TARGET_CLASS_IDS = {}
IMG_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp"}


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def coco_bbox_to_yolo(bbox, img_w, img_h):
    """COCO [x, y, w, h] → YOLO [cx, cy, w, h] 归一化"""
    x, y, w, h = bbox
    cx = (x + w / 2) / img_w
    cy = (y + h / 2) / img_h
    nw = w / img_w
    nh = h / img_h
    return cx, cy, nw, nh


def resolve_mapping(mapping_rules, key):
    """Resolve mapping key — handles string, int, and '*'>
    wildcard keys"""
    # Try as-is
    if key in mapping_rules:
        return mapping_rules[key]
    # Try string version
    if str(key) in mapping_rules:
        return mapping_rules[str(key)]
    # Try int version
    if isinstance(key, str):
        try:
            ik = int(key)
            if ik in mapping_rules:
                return mapping_rules[ik]
        except ValueError:
            pass
    # Wildcard fallback
    if "*" in mapping_rules:
        return mapping_rules["*"]
    return None


def write_merged_image(src_img, merged_img, merged_lbl, unique_stem, lines):
    """Copy image and write label file into merged/"""
    merged_img.mkdir(parents=True, exist_ok=True)
    merged_lbl.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src_img, merged_img / f"{unique_stem}.jpg")
    if lines:
        with open(merged_lbl / f"{unique_stem}.txt", "w") as f:
            f.writelines(lines)


def _convert_coco_core(ds_path, ann_path, mapping_rules, processed_dir, stem_prefix, img_subdir=None, per_class_cap=None):
    """Core COCO → YOLO conversion logic, shared by all COCO variants.

    img_subdir: if set, images live in ds_path / img_subdir / file_name.
                If None, file_name is relative to ds_path directly.
    per_class_cap: dict {target_class_name: max_boxes_per_image}, applied after mapping.
    """
    if not ann_path.exists():
        return 0, {}

    with open(ann_path, "r", encoding="utf-8") as f:
        coco = json.load(f)

    id_to_img = {}
    for img_info in coco.get("images", []):
        id_to_img[img_info["id"]] = {
            "file_name": img_info["file_name"],
            "width": img_info["width"],
            "height": img_info["height"],
        }

    id_to_cat = {}
    for cat in coco.get("categories", []):
        id_to_cat[cat["id"]] = cat["name"]

    img_annotations = defaultdict(list)
    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}

    for ann in coco.get("annotations", []):
        stats["total"] += 1
        orig_cat = id_to_cat.get(ann["category_id"], "")
        target_label = resolve_mapping(mapping_rules, orig_cat)
        if target_label is None or target_label == "discard":
            stats["discarded"] += 1
            continue

        img_info = id_to_img.get(ann["image_id"])
        if img_info is None:
            stats["discarded"] += 1
            continue

        cx, cy, nw, nh = coco_bbox_to_yolo(ann["bbox"], img_info["width"], img_info["height"])
        cx, cy = max(0.0, min(1.0, cx)), max(0.0, min(1.0, cy))
        nw, nh = max(0.0, min(1.0, nw)), max(0.0, min(1.0, nh))
        if nw <= 0.001 or nh <= 0.001:
            stats["discarded"] += 1
            continue

        class_id = TARGET_CLASS_IDS[target_label]
        img_annotations[ann["image_id"]].append(
            f"{class_id} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n"
        )
        stats["kept"] += 1
        stats["per_class"][target_label] += 1

    # Apply per-class cap — limits boxes per image for specified classes
    # Reverse TARGET_CLASS_IDS to get class_name from class_id
    id_to_name = {v: k for k, v in TARGET_CLASS_IDS.items()}
    if per_class_cap:
        capped_total = 0
        for img_id, anns in img_annotations.items():
            # Group annotations by target class name
            by_class = defaultdict(list)
            for line in anns:
                cls_id = int(line.split()[0])
                name = id_to_name.get(cls_id, "unknown")
                by_class[name].append(line)
            # Cap each class
            new_anns = []
            for cls_name, lines in by_class.items():
                cap = per_class_cap.get(cls_name)
                if cap is not None and len(lines) > cap:
                    import random
                    random.shuffle(lines)
                    new_anns.extend(lines[:cap])
                    capped_total += len(lines) - cap
                else:
                    new_anns.extend(lines)
            img_annotations[img_id] = new_anns
        stats["kept"] -= capped_total

    merged_img = processed_dir / "merged" / "images"
    merged_lbl = processed_dir / "merged" / "labels"

    written = 0
    for img_id, img_info in id_to_img.items():
        fn = img_info["file_name"]
        if img_subdir:
            src_img = ds_path / img_subdir / fn
        else:
            src_img = ds_path / fn
        if not src_img.exists():
            continue
        stem = Path(fn).stem
        unique_stem = f"{stem_prefix}_{stem}"
        lines = img_annotations.get(img_id, [])
        write_merged_image(src_img, merged_img, merged_lbl, unique_stem, lines)
        written += 1

    return written, stats


def convert_taco_coco(ds_config, mapping_rules, processed_dir):
    """Convert TACO COCO format — images in batch_X/ subdirs"""
    raw_path = ds_config["path"]
    ds_path = Path(raw_path)
    if not ds_path.is_absolute():
        ds_path = (PROJECT_ROOT / ds_path).resolve()
    ann_path = ds_path / ds_config["annotation_file"]
    return _convert_coco_core(ds_path, ann_path, mapping_rules, processed_dir,
                              stem_prefix="taco")


def convert_coco_flat(ds_config, mapping_rules, processed_dir):
    """Convert COCO format — images in images/ subdirectory, flat filenames"""
    raw_path = ds_config["path"]
    ds_path = Path(raw_path)
    if not ds_path.is_absolute():
        ds_path = (PROJECT_ROOT / ds_path).resolve()
    ann_path = ds_path / ds_config["annotation_file"]
    return _convert_coco_core(ds_path, ann_path, mapping_rules, processed_dir,
                              stem_prefix="dat", img_subdir="images",
                              per_class_cap=ds_config.get("per_class_cap"))


def convert_classify_csv(ds_config, mapping_rules, processed_dir):
    """Convert classification CSV format to YOLO txt (full-image boxes).

    Format: each .txt file contains 'filename.jpg, class_id'
    The image IS the object → bbox = whole image.
    """
    ds_name = ds_config["name"]
    raw_path = ds_config["path"]
    ds_path = Path(raw_path)
    if not ds_path.is_absolute():
        ds_path = (PROJECT_ROOT / ds_path).resolve()

    data_dir = ds_path / "train_data"
    if not data_dir.exists():
        print(f"  [SKIP] {ds_name}: {data_dir} not found")
        return 0, {}

    txt_files = sorted([f for f in os.listdir(str(data_dir)) if f.endswith(".txt")])

    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    written = 0

    merged_img = processed_dir / "merged" / "images"
    merged_lbl = processed_dir / "merged" / "labels"

    for tf in txt_files:
        lbl_path = data_dir / tf
        with open(lbl_path, "r", encoding="utf-8") as f:
            content = f.read().strip()

        if not content:
            continue

        # Parse "filename.jpg, class_id" or "filename.jpg class_id"
        if "," in content:
            img_name, cls_str = content.rsplit(",", 1)
        else:
            parts = content.rsplit(" ", 1)
            img_name, cls_str = parts[0], parts[1]

        cls_str = cls_str.strip()
        img_name = img_name.strip()
        img_path = data_dir / img_name

        if not img_path.exists():
            continue

        stats["total"] += 1
        source_cls_id = int(cls_str)
        target_label = resolve_mapping(mapping_rules, source_cls_id)

        if target_label is None:
            stats["discarded"] += 1
            continue

        class_id = TARGET_CLASS_IDS[target_label]
        # Full-image detection box
        line = f"{class_id} 0.5 0.5 1.0 1.0\n"

        unique_stem = f"gc_{Path(img_name).stem}"
        write_merged_image(img_path, merged_img, merged_lbl, unique_stem, [line])
        stats["kept"] += 1
        stats["per_class"][target_label] += 1
        written += 1

    return written, stats


def convert_yolo_remap(ds_config, mapping_rules, processed_dir):
    """Convert YOLO txt dataset by remapping class IDs."""
    ds_name = ds_config["name"]
    raw_path = ds_config["path"]
    ds_path = Path(raw_path)
    if not ds_path.is_absolute():
        ds_path = (PROJECT_ROOT / ds_path).resolve()

    merged_img = processed_dir / "merged" / "images"
    merged_lbl = processed_dir / "merged" / "labels"

    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    total_images = 0

    for split_name in ["train", "valid", "val", "test"]:
        split_img_dir = ds_path / "images" / split_name
        split_lbl_dir = ds_path / "labels" / split_name
        if not split_img_dir.exists() or not split_lbl_dir.exists():
            continue

        for lbl_file in sorted(split_lbl_dir.iterdir()):
            if not lbl_file.suffix == ".txt":
                continue

            stem = lbl_file.stem
            img_file = None
            for ext in IMG_EXTENSIONS:
                candidate = split_img_dir / (stem + ext)
                if candidate.exists():
                    img_file = candidate
                    break
            if img_file is None:
                continue

            new_lines = []
            with open(lbl_file, "r") as f:
                for line in f:
                    parts = line.strip().split()
                    if not parts:
                        continue
                    stats["total"] += 1
                    old_cls = int(parts[0])
                    target_label = resolve_mapping(mapping_rules, old_cls)
                    if target_label is None or target_label == "discard":
                        stats["discarded"] += 1
                        continue
                    new_cls = TARGET_CLASS_IDS[target_label]
                    new_lines.append(f"{new_cls} {' '.join(parts[1:])}\n")
                    stats["kept"] += 1
                    stats["per_class"][target_label] += 1

            unique_stem = f"{ds_name}_{split_name}_{stem}"
            write_merged_image(img_file, merged_img, merged_lbl, unique_stem, new_lines)
            total_images += 1

    return total_images, stats


def convert_folder_class(ds_config, mapping_rules, processed_dir):
    """Convert folder-based classification to YOLO txt (full-image boxes).

    Each subfolder name IS the class label. All images inside get a
    single full-image detection box of that class.
    """
    ds_name = ds_config["name"]
    raw_path = ds_config["path"]
    ds_path = Path(raw_path)
    if not ds_path.is_absolute():
        ds_path = (PROJECT_ROOT / ds_path).resolve()

    if not ds_path.exists():
        print(f"  [SKIP] {ds_name}: path not found")
        return 0, {}

    merged_img = processed_dir / "merged" / "images"
    merged_lbl = processed_dir / "merged" / "labels"

    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    total_images = 0

    for class_dir in sorted(ds_path.iterdir()):
        if not class_dir.is_dir():
            continue
        dir_name = class_dir.name
        # Skip macOS metadata
        if dir_name.startswith(".") or dir_name == "__MACOSX":
            continue

        target_label = resolve_mapping(mapping_rules, dir_name.lower())
        if target_label is None or target_label == "discard":
            continue
        class_id = TARGET_CLASS_IDS[target_label]

        for img_file in sorted(class_dir.iterdir()):
            if img_file.suffix.lower() not in IMG_EXTENSIONS:
                continue
            stats["total"] += 1
            line = f"{class_id} 0.5 0.5 1.0 1.0\n"
            unique_stem = f"tn_{dir_name}_{img_file.stem}"
            write_merged_image(img_file, merged_img, merged_lbl, unique_stem, [line])
            stats["kept"] += 1
            stats["per_class"][target_label] += 1
            total_images += 1

    return total_images, stats


def convert_csv_classify(ds_config, mapping_rules, processed_dir):
    """Convert CSV classification to YOLO txt (full-image boxes).

    CSV format: image,label (e.g. images/0.jpg,maclura_pomifera)
    All labels mapped via rules with * wildcard support.
    """
    import csv

    ds_name = ds_config["name"]
    raw_path = ds_config["path"]
    ds_path = Path(raw_path)
    if not ds_path.is_absolute():
        ds_path = (PROJECT_ROOT / ds_path).resolve()

    csv_path = ds_path / "train.csv"
    if not csv_path.exists():
        print(f"  [SKIP] {ds_name}: train.csv not found")
        return 0, {}

    merged_img = processed_dir / "merged" / "images"
    merged_lbl = processed_dir / "merged" / "labels"

    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    total_images = 0

    with open(csv_path, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        for row in reader:
            img_rel = row.get("image", "")
            label_raw = row.get("label", "").strip()
            if not label_raw:
                stats["discarded"] += 1
                continue

            # Extract filename from path (handles "images/0.jpg" and "0.jpg")
            img_name = os.path.basename(img_rel)
            img_path = ds_path / "images" / img_name
            if not img_path.exists():
                stats["discarded"] += 1
                continue

            stats["total"] += 1
            target_label = resolve_mapping(mapping_rules, label_raw)
            if target_label is None or target_label == "discard":
                stats["discarded"] += 1
                continue

            class_id = TARGET_CLASS_IDS[target_label]
            line = f"{class_id} 0.5 0.5 1.0 1.0\n"
            unique_stem = f"leaf_{img_name.split('.')[0]}"
            write_merged_image(img_path, merged_img, merged_lbl, unique_stem, [line])
            stats["kept"] += 1
            stats["per_class"][target_label] += 1
            total_images += 1

    return total_images, stats


def convert_voc_xml(ds_config, mapping_rules, processed_dir):
    """Convert Pascal VOC XML to YOLO txt.

    Flat structure: images/ and .xml files in same directory.
    XML <bndbox> → YOLO normalized box.
    """
    import xml.etree.ElementTree as ET

    ds_name = ds_config["name"]
    raw_path = ds_config["path"]
    ds_path = Path(raw_path)
    if not ds_path.is_absolute():
        ds_path = (PROJECT_ROOT / ds_path).resolve()

    if not ds_path.exists():
        print(f"  [SKIP] {ds_name}: path not found")
        return 0, {}

    merged_img = processed_dir / "merged" / "images"
    merged_lbl = processed_dir / "merged" / "labels"
    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}

    xml_files = sorted([f for f in os.listdir(str(ds_path)) if f.endswith(".xml")])
    total_images = 0

    for xf in xml_files:
        tree = ET.parse(str(ds_path / xf))
        root = tree.getroot()

        filename = root.findtext("filename", "")
        if not filename:
            continue

        # Find image file
        img_path = ds_path / filename
        if not img_path.exists():
            continue

        # Get image size
        sz = root.find("size")
        img_w = int(sz.findtext("width", "640"))
        img_h = int(sz.findtext("height", "640"))

        new_lines = []
        for obj in root.findall("object"):
            name = obj.findtext("name", "").strip()
            stats["total"] += 1

            target_label = resolve_mapping(mapping_rules, name)
            if target_label is None or target_label == "discard":
                stats["discarded"] += 1
                continue

            bbox = obj.find("bndbox")
            if bbox is None:
                stats["discarded"] += 1
                continue
            xmin = float(bbox.findtext("xmin", "0"))
            ymin = float(bbox.findtext("ymin", "0"))
            xmax = float(bbox.findtext("xmax", str(img_w)))
            ymax = float(bbox.findtext("ymax", str(img_h)))

            # VOC [xmin,ymin,xmax,ymax] → YOLO [cx,cy,w,h] normalized
            cx = ((xmin + xmax) / 2) / img_w
            cy = ((ymin + ymax) / 2) / img_h
            nw = (xmax - xmin) / img_w
            nh = (ymax - ymin) / img_h
            cx, cy = max(0.0, min(1.0, cx)), max(0.0, min(1.0, cy))
            nw, nh = max(0.0, min(1.0, nw)), max(0.0, min(1.0, nh))

            if nw <= 0.001 or nh <= 0.001:
                stats["discarded"] += 1
                continue

            class_id = TARGET_CLASS_IDS[target_label]
            new_lines.append(f"{class_id} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")
            stats["kept"] += 1
            stats["per_class"][target_label] += 1

        unique_stem = f"pdl_{Path(filename).stem}"
        write_merged_image(img_path, merged_img, merged_lbl, unique_stem, new_lines)
        total_images += 1

    return total_images, stats


def split_dataset(processed_dir):
    """将 merged 按 7:1.5:1.5 划分为 train/val/test"""
    merged_img = processed_dir / "merged" / "images"
    merged_lbl = processed_dir / "merged" / "labels"
    if not merged_img.exists():
        print("  [WARN] merged/ empty, skip split")
        return

    pairs = []
    for img_file in sorted(merged_img.iterdir()):
        if img_file.suffix.lower() not in IMG_EXTENSIONS:
            continue
        lbl_file = merged_lbl / f"{img_file.stem}.txt"
        if lbl_file.exists():
            pairs.append((img_file, lbl_file))

    random.seed(42)
    random.shuffle(pairs)
    n = len(pairs)

    train_end = int(n * 0.70)
    val_end = int(n * 0.85)

    splits = {
        "train": pairs[:train_end],
        "val": pairs[train_end:val_end],
        "test": pairs[val_end:],
    }

    for split_name, items in splits.items():
        img_dir = processed_dir / split_name / "images"
        lbl_dir = processed_dir / split_name / "labels"
        if img_dir.exists():
            shutil.rmtree(img_dir)
        if lbl_dir.exists():
            shutil.rmtree(lbl_dir)
        img_dir.mkdir(parents=True, exist_ok=True)
        lbl_dir.mkdir(parents=True, exist_ok=True)
        for img, lbl in items:
            shutil.copy2(img, img_dir / img.name)
            shutil.copy2(lbl, lbl_dir / lbl.name)

    print(f"  Split: train={len(splits['train'])}, val={len(splits['val'])}, test={len(splits['test'])}")


def main():
    parser = argparse.ArgumentParser(description="数据集准备: 标签映射 + 格式转换 + 划分")
    parser.add_argument("--skip", nargs="*", default=[], help="跳过的数据源名称")
    args = parser.parse_args()

    global TARGET_CLASS_IDS
    label_mapping = load_yaml(PROJECT_ROOT / "configs/label_mapping.yaml")
    TARGET_CLASS_IDS = {v: k for k, v in label_mapping["target_classes"].items()}

    source_config = load_yaml(PROJECT_ROOT / "configs/source_datasets.yaml")
    processed_dir = PROJECT_ROOT.parent / "数据集" / "processed"

    # Clean and recreate
    if processed_dir.exists():
        shutil.rmtree(processed_dir)
    merged_img = processed_dir / "merged" / "images"
    merged_lbl = processed_dir / "merged" / "labels"
    merged_img.mkdir(parents=True)
    merged_lbl.mkdir(parents=True)

    print("=" * 60)
    print("  Dataset Preparation — Multi-source → YOLOv8 txt")
    print("=" * 60)

    all_stats = {}
    total_images = 0

    handlers = {
        "coco_taco": convert_taco_coco,
        "coco_flat": convert_coco_flat,
        "classify_csv": convert_classify_csv,
        "csv_classify": convert_csv_classify,
        "yolo_remap": convert_yolo_remap,
        "folder_class": convert_folder_class,
        "voc_xml": convert_voc_xml,
        "yolo_txt": lambda ds, mapping, pd: (0, {}),
    }

    for ds in source_config["datasets"]:
        name = ds["name"]
        if name in args.skip:
            print(f"\n[{name}] SKIPPED (--skip)")
            continue
        if ds.get("enabled") is False:
            print(f"\n[{name}] SKIPPED (enabled=false)")
            continue

        fmt = ds["format"]
        mapping_rules = label_mapping.get("mappings", {}).get(ds.get("mapping", ""), {})
        handler = handlers.get(fmt)

        print(f"\n[{name}] format={fmt}")
        if handler:
            n, stats = handler(ds, mapping_rules, processed_dir)
        else:
            print(f"  [SKIP] unknown format: {fmt}")
            continue

        total_images += n
        all_stats[name] = stats
        if isinstance(stats, dict) and stats:
            print(f"  Images: {n}")
            print(f"  Annotations: {stats['kept']} kept / {stats['discarded']} discarded / {stats['total']} total")
        elif n:
            print(f"  Images: {n}")

    # Split
    print("\n[Splitting] 7:1.5:1.5 ...")
    split_dataset(processed_dir)

    # Gap analysis
    print("\n" + "=" * 60)
    print("  Dataset vs. Project Target Gap Analysis")
    print("=" * 60)

    class_counts = defaultdict(int)
    for split_name in ["train", "val", "test"]:
        lbl_dir = processed_dir / split_name / "labels"
        if lbl_dir.exists():
            for lbl_file in lbl_dir.iterdir():
                with open(lbl_file) as f:
                    for line in f:
                        cid = int(line.split()[0])
                        class_counts[cid] += 1

    target_classes = label_mapping["target_classes"]
    total_boxes = sum(class_counts.values())

    print(f"\n  {'Class':<20} {'Boxes':>8} {'%':>7}  Status")
    print(f"  {'-'*20} {'-'*8} {'-'*7}  {'-'*20}")

    target_min = 500  # Minimum boxes for viable training
    for cid in sorted(target_classes):
        name = target_classes[cid]
        count = class_counts.get(cid, 0)
        pct = count / total_boxes * 100 if total_boxes else 0
        if count >= target_min:
            st = "OK"
        elif count >= 100:
            st = "WEAK (need more)"
        elif count > 0:
            st = "CRITICAL (very few)"
        else:
            st = "MISSING (none)"
        print(f"  {name:<20} {count:>8} {pct:>6.1f}%  {st}")

    print(f"  {'-'*20} {'-'*8} {'-'*7}")
    print(f"  {'TOTAL':<20} {total_boxes:>8}")

    print("\n" + "=" * 60)
    print(f"  Done — {total_images} images → train/val/test")
    print(f"  Output: {processed_dir}")
    print("=" * 60)


if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()
    main()
