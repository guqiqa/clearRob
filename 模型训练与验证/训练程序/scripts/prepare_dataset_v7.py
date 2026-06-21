#!/usr/bin/env python3
"""
V7 数据集准备 — 细粒度标签映射 + 格式转换 + 数据划分 (支持 20 细类)

用法:
  python scripts/prepare_dataset_v7.py
  python scripts/prepare_dataset_v7.py --skip taco
"""

import argparse, json, os, random, shutil, sys, csv
from collections import defaultdict
from pathlib import Path
import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
FINE_CLASS_IDS = {}       # fine class name → ID
IMG_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp"}

def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)

def resolve_mapping(mapping_rules, key):
    if key in mapping_rules: return mapping_rules[key]
    if str(key) in mapping_rules: return mapping_rules[str(key)]
    if isinstance(key, str):
        try:
            ik = int(key)
            if ik in mapping_rules: return mapping_rules[ik]
        except ValueError: pass
    return None

def coco_bbox_to_yolo(bbox, w, h):
    x, y, bw, bh = bbox
    cx = max(0, min(1, (x + bw/2) / w))
    cy = max(0, min(1, (y + bh/2) / h))
    nw = max(0, min(1, bw / w))
    nh = max(0, min(1, bh / h))
    return cx, cy, nw, nh

def write_merged_image(src, merged_img, merged_lbl, stem, lines):
    merged_img.mkdir(parents=True, exist_ok=True)
    merged_lbl.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, merged_img / f"{stem}.jpg")
    if lines:
        with open(merged_lbl / f"{stem}.txt", "w") as f:
            f.writelines(lines)

def _coco_core(ds_path, ann_path, mapping_rules, proc_dir, prefix, img_subdir=None, per_class_cap=None):
    if not ann_path.exists(): return 0, {}
    with open(ann_path, "r", encoding="utf-8") as f:
        coco = json.load(f)
    id_to_img = {i["id"]: {"fn": i["file_name"], "w": i["width"], "h": i["height"]} for i in coco.get("images", [])}
    id_to_cat = {c["id"]: c["name"] for c in coco.get("categories", [])}
    img_anns = defaultdict(list)
    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    id_to_fine = {v: k for k, v in FINE_CLASS_IDS.items()}

    for ann in coco.get("annotations", []):
        stats["total"] += 1
        orig = id_to_cat.get(ann["category_id"], "")
        fine_name = resolve_mapping(mapping_rules, orig)
        if fine_name is None or fine_name == "discard": stats["discarded"] += 1; continue
        info = id_to_img.get(ann["image_id"])
        if info is None: stats["discarded"] += 1; continue
        cx, cy, nw, nh = coco_bbox_to_yolo(ann["bbox"], info["w"], info["h"])
        if nw <= 0.001 or nh <= 0.001: stats["discarded"] += 1; continue
        fid = FINE_CLASS_IDS[fine_name]
        img_anns[ann["image_id"]].append(f"{fid} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")
        stats["kept"] += 1; stats["per_class"][fine_name] += 1

    # Apply per_class_cap
    if per_class_cap:
        capped = 0
        for img_id, lines in list(img_anns.items()):
            by_cls = defaultdict(list)
            for line in lines:
                by_cls[id_to_fine[int(line.split()[0])]].append(line)
            new_lines = []
            for cls_name, ls in by_cls.items():
                cap = per_class_cap.get(cls_name)
                if cap and len(ls) > cap:
                    random.shuffle(ls); new_lines.extend(ls[:cap]); capped += len(ls) - cap
                else:
                    new_lines.extend(ls)
            img_anns[img_id] = new_lines
        stats["kept"] -= capped

    merged_img = proc_dir / "merged" / "images"
    merged_lbl = proc_dir / "merged" / "labels"
    written = 0
    for img_id, info in id_to_img.items():
        src = ds_path / (f"{img_subdir}/{info['fn']}" if img_subdir else info["fn"])
        if not src.exists(): continue
        stem = f"{prefix}_{Path(info['fn']).stem}"
        write_merged_image(src, merged_img, merged_lbl, stem, img_anns.get(img_id, []))
        written += 1
    return written, stats

def convert_coco_taco(ds, m, pd):
    p = (PROJECT_ROOT / ds["path"]).resolve()
    return _coco_core(p, p / ds["annotation_file"], m, pd, "taco")

def convert_coco_flat(ds, m, pd):
    p = (PROJECT_ROOT / ds["path"]).resolve()
    return _coco_core(p, p / ds["annotation_file"], m, pd, "dat", "images", ds.get("per_class_cap"))

def convert_classify_csv(ds, m, pd):
    p = (PROJECT_ROOT / ds["path"]).resolve()
    ddir = p / "train_data"
    if not ddir.exists(): return 0, {}
    merged_img = pd / "merged" / "images"
    merged_lbl = pd / "merged" / "labels"
    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    imgs = 0

    for tf in sorted(ddir.iterdir()):
        if not tf.suffix == ".txt": continue
        with open(tf, encoding="utf-8") as f:
            content = f.read().strip()
        if not content: continue
        if "," in content:
            img_name, cls_str = content.rsplit(",", 1)
        else:
            parts = content.rsplit(" ", 1)
            img_name, cls_str = parts[0], parts[1]
        img_name = img_name.strip(); cls_str = cls_str.strip()
        ip = ddir / img_name
        if not ip.exists(): continue
        stats["total"] += 1
        fine_name = resolve_mapping(m, int(cls_str))
        if fine_name is None or fine_name == "discard": stats["discarded"] += 1; continue
        fid = FINE_CLASS_IDS[fine_name]
        line = f"{fid} 0.5 0.5 1.0 1.0\n"
        write_merged_image(ip, merged_img, merged_lbl, f"gc_{Path(img_name).stem}", [line])
        stats["kept"] += 1; stats["per_class"][fine_name] += 1
        imgs += 1
    return imgs, stats

def convert_yolo_remap(ds, m, pd):
    p = (PROJECT_ROOT / ds["path"]).resolve()
    merged_img = pd / "merged" / "images"
    merged_lbl = pd / "merged" / "labels"
    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    imgs = 0

    for split in ["train", "valid", "val", "test"]:
        img_dir = p / "images" / split
        lbl_dir = p / "labels" / split
        if not img_dir.exists() or not lbl_dir.exists(): continue
        for lf in sorted(lbl_dir.iterdir()):
            if lf.suffix != ".txt": continue
            stem = lf.stem
            img_file = None
            for ext in IMG_EXTENSIONS:
                c = img_dir / (stem + ext)
                if c.exists(): img_file = c; break
            if img_file is None: continue
            new_lines = []
            with open(lf) as f:
                for line in f:
                    parts = line.strip().split()
                    if not parts: continue
                    stats["total"] += 1
                    fine_name = resolve_mapping(m, int(parts[0]))
                    if fine_name is None or fine_name == "discard": stats["discarded"] += 1; continue
                    fid = FINE_CLASS_IDS[fine_name]
                    new_lines.append(f"{fid} {' '.join(parts[1:])}\n")
                    stats["kept"] += 1; stats["per_class"][fine_name] += 1
            write_merged_image(img_file, merged_img, merged_lbl, f"{ds['name']}_{split}_{stem}", new_lines)
            imgs += 1
    return imgs, stats

def convert_csv_classify(ds, m, pd):
    p = (PROJECT_ROOT / ds["path"]).resolve()
    csv_path = p / "train.csv"
    if not csv_path.exists(): return 0, {}
    merged_img = pd / "merged" / "images"
    merged_lbl = pd / "merged" / "labels"
    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    imgs = 0
    with open(csv_path, encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            img_rel = row.get("image", "")
            lbl = row.get("label", "").strip()
            if not lbl: stats["discarded"] += 1; continue
            ip = p / "images" / os.path.basename(img_rel)
            if not ip.exists(): stats["discarded"] += 1; continue
            stats["total"] += 1
            fine_name = resolve_mapping(m, lbl)
            if fine_name is None or fine_name == "discard": stats["discarded"] += 1; continue
            fid = FINE_CLASS_IDS[fine_name]
            line = f"{fid} 0.5 0.5 1.0 1.0\n"
            write_merged_image(ip, merged_img, merged_lbl, f"leaf_{Path(img_rel).stem}", [line])
            stats["kept"] += 1; stats["per_class"][fine_name] += 1
            imgs += 1
    return imgs, stats

def convert_folder_class(ds, m, pd):
    """普通 folder_class: 文件夹名 = class label, 无平衡"""
    p = (PROJECT_ROOT / ds["path"]).resolve()
    if not p.exists(): return 0, {}
    merged_img = pd / "merged" / "images"
    merged_lbl = pd / "merged" / "labels"
    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    imgs = 0
    for cd in sorted(p.iterdir()):
        if not cd.is_dir() or cd.name.startswith(".") or cd.name == "__MACOSX": continue
        fine_name = resolve_mapping(m, cd.name.lower())
        if fine_name is None or fine_name == "discard": continue
        fid = FINE_CLASS_IDS[fine_name]
        for img_file in sorted(cd.iterdir()):
            if img_file.suffix.lower() not in IMG_EXTENSIONS: continue
            stats["total"] += 1
            line = f"{fid} 0.5 0.5 1.0 1.0\n"
            write_merged_image(img_file, merged_img, merged_lbl, f"tn_{cd.name}_{img_file.stem}", [line])
            stats["kept"] += 1; stats["per_class"][fine_name] += 1
            imgs += 1
    return imgs, stats


def convert_folder_balanced(ds, m, pd):
    """平衡版 folder_class: 支持 samples_per_class 控制 + Recyclable 四路拆分"""
    p = (PROJECT_ROOT / ds["path"]).resolve()
    if not p.exists(): return 0, {}
    merged_img = pd / "merged" / "images"
    merged_lbl = pd / "merged" / "labels"
    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    imgs = 0
    max_per = ds.get("samples_per_class", None)

    recycled_subclasses = ["rec_plastic", "rec_metal", "rec_glass", "rec_paper"]

    for cd in sorted(p.iterdir()):
        if not cd.is_dir() or cd.name.startswith(".") or cd.name == "__MACOSX": continue
        dir_name = cd.name
        fine_name_raw = resolve_mapping(m, dir_name)

        if fine_name_raw is None or fine_name_raw == "discard": continue

        # Get all valid images in this folder
        all_imgs = sorted([f for f in cd.iterdir() if f.suffix.lower() in IMG_EXTENSIONS])
        stats["total"] += len(all_imgs)
        random.shuffle(all_imgs)

        # Apply per-class sampling
        if max_per and len(all_imgs) > max_per:
            all_imgs = all_imgs[:max_per]

        for img_file in all_imgs:
            # Handle @split_recyclable — evenly distribute across 4 rec_* subclasses
            if fine_name_raw == "@split_recyclable":
                # Round-robin across 4 sub-classes
                idx = len([x for x in stats["per_class"].keys() if x in recycled_subclasses]) % 4
                # Actually use a hash-based approach for deterministic split
                h = hash(img_file.name) % 4
                fine_name = recycled_subclasses[h]
            else:
                fine_name = fine_name_raw

            fid = FINE_CLASS_IDS[fine_name]
            line = f"{fid} 0.5 0.5 1.0 1.0\n"
            write_merged_image(img_file, merged_img, merged_lbl, f"gb_{dir_name}_{img_file.stem}", [line])
            stats["kept"] += 1
            stats["per_class"][fine_name] += 1
            imgs += 1
    return imgs, stats

def convert_voc_xml(ds, m, pd):
    import xml.etree.ElementTree as ET
    p = (PROJECT_ROOT / ds["path"]).resolve()
    if not p.exists(): return 0, {}
    merged_img = pd / "merged" / "images"
    merged_lbl = pd / "merged" / "labels"
    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    imgs = 0
    for xf in sorted(os.listdir(str(p))):
        if not xf.endswith(".xml"): continue
        tree = ET.parse(str(p / xf))
        root = tree.getroot()
        fn = root.findtext("filename", "")
        ip = p / fn
        if not ip.exists(): continue
        sz = root.find("size")
        iw = int(sz.findtext("width", "640"))
        ih = int(sz.findtext("height", "640"))
        new_lines = []
        for obj in root.findall("object"):
            name = obj.findtext("name", "").strip()
            stats["total"] += 1
            fine_name = resolve_mapping(m, name)
            if fine_name is None or fine_name == "discard": stats["discarded"] += 1; continue
            bb = obj.find("bndbox")
            if bb is None: stats["discarded"] += 1; continue
            xmin = float(bb.findtext("xmin", "0")); ymin = float(bb.findtext("ymin", "0"))
            xmax = float(bb.findtext("xmax", str(iw))); ymax = float(bb.findtext("ymax", str(ih)))
            cx = max(0, min(1, (xmin+xmax)/2/iw))
            cy = max(0, min(1, (ymin+ymax)/2/ih))
            nw = max(0, min(1, (xmax-xmin)/iw))
            nh = max(0, min(1, (ymax-ymin)/ih))
            if nw <= 0.001 or nh <= 0.001: stats["discarded"] += 1; continue
            fid = FINE_CLASS_IDS[fine_name]
            new_lines.append(f"{fid} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")
            stats["kept"] += 1; stats["per_class"][fine_name] += 1
        write_merged_image(ip, merged_img, merged_lbl, f"pdl_{Path(fn).stem}", new_lines)
        imgs += 1
    return imgs, stats

def split_dataset(proc_dir):
    merged_img = proc_dir / "merged" / "images"
    merged_lbl = proc_dir / "merged" / "labels"
    if not merged_img.exists(): return
    pairs = []
    for img_file in sorted(merged_img.iterdir()):
        if img_file.suffix.lower() not in IMG_EXTENSIONS: continue
        lf = merged_lbl / f"{img_file.stem}.txt"
        if lf.exists(): pairs.append((img_file, lf))
    random.seed(42); random.shuffle(pairs)
    n = len(pairs)
    splits = {"train": pairs[:int(n*0.7)], "val": pairs[int(n*0.7):int(n*0.85)], "test": pairs[int(n*0.85):]}
    for sn, items in splits.items():
        img_dir = proc_dir / sn / "images"; lbl_dir = proc_dir / sn / "labels"
        if img_dir.exists(): shutil.rmtree(img_dir)
        if lbl_dir.exists(): shutil.rmtree(lbl_dir)
        img_dir.mkdir(parents=True); lbl_dir.mkdir(parents=True)
        for img, lbl in items:
            shutil.copy2(img, img_dir / img.name)
            shutil.copy2(lbl, lbl_dir / lbl.name)
    print(f"  Split: train={len(splits['train'])}, val={len(splits['val'])}, test={len(splits['test'])}")

def main():
    parser = argparse.ArgumentParser(description="V7 细粒度数据集准备")
    parser.add_argument("--skip", nargs="*", default=[])
    args = parser.parse_args()

    global FINE_CLASS_IDS
    fine_cfg = load_yaml(PROJECT_ROOT / "configs/fine_labels.yaml")
    FINE_CLASS_IDS = {v: k for k, v in fine_cfg["fine_classes"].items()}

    mapping = load_yaml(PROJECT_ROOT / "configs/label_mapping_v7.yaml")
    sources = load_yaml(PROJECT_ROOT / "configs/source_datasets_v7.yaml")
    proc_dir = PROJECT_ROOT.parent / "数据集" / "processed_v7"

    if proc_dir.exists(): shutil.rmtree(proc_dir)
    (proc_dir / "merged" / "images").mkdir(parents=True)
    (proc_dir / "merged" / "labels").mkdir(parents=True)

    print("=" * 60)
    print("  V7 Fine-Grained Dataset Preparation (20 classes)")
    print("=" * 60)

    handlers = {
        "coco_taco": convert_coco_taco, "coco_flat": convert_coco_flat,
        "classify_csv": convert_classify_csv, "csv_classify": convert_csv_classify,
        "yolo_remap": convert_yolo_remap,
        "folder_class": convert_folder_class,
        "folder_balanced": convert_folder_balanced,
        "voc_xml": convert_voc_xml, "yolo_txt": lambda *a: (0, {}),
    }
    total_imgs = 0

    for ds in sources["datasets"]:
        name = ds["name"]
        if name in args.skip:
            print(f"\n[{name}] SKIPPED (--skip)"); continue
        if ds.get("enabled") is False:
            print(f"\n[{name}] SKIPPED (disabled)"); continue

        fmt = ds["format"]
        rules = mapping["mappings"].get(ds.get("mapping", ""), {})
        handler = handlers.get(fmt)
        if not handler: print(f"\n[{name}] UNKNOWN format={fmt}"); continue

        print(f"\n[{name}] format={fmt}")
        n, stats = handler(ds, rules, proc_dir)
        total_imgs += n
        if isinstance(stats, dict) and stats:
            print(f"  Images: {n} | Kept: {stats['kept']} / Discarded: {stats['discarded']} / Total: {stats['total']}")

    print("\n[Splitting] 7:1.5:1.5 ...")
    split_dataset(proc_dir)

    # Per-class stats
    class_counts = defaultdict(int)
    train_lbl = proc_dir / "train" / "labels"
    if train_lbl.exists():
        for lf in train_lbl.iterdir():
            with open(lf) as f:
                for line in f:
                    class_counts[int(line.split()[0])] += 1

    print("\n" + "=" * 60)
    print("  Fine-Grained Class Distribution (train)")
    print("=" * 60)
    id_to_name = {v: k for k, v in FINE_CLASS_IDS.items()}
    coarse_names = fine_cfg["coarse_names"]

    for fid in sorted(id_to_name):
        name = id_to_name[fid]
        cnt = class_counts.get(fid, 0)
        cid = fine_cfg["fine_to_coarse"][fid]
        cname = coarse_names[cid]
        bar = "=" * min(cnt // 50, 15) if cnt > 0 else ""
        status = "OK" if cnt >= 200 else ("WEAK" if cnt > 0 else "MISSING")
        print(f"  {fid:>2}: {name:<20} {cnt:>6} |{bar:<15}| → {cname:<18} {status}")
    print(f"  {'TOTAL':>24} {sum(class_counts.values()):>6}")

    # Coarse distribution
    coarse_counts = defaultdict(int)
    for fid, cnt in class_counts.items():
        cid = fine_cfg["fine_to_coarse"].get(fid, 0)
        coarse_counts[cid] += cnt

    print(f"\n  === Merged to 7 coarse classes === ")
    for cid in range(7):
        name = coarse_names[cid]
        cnt = coarse_counts.get(cid, 0)
        print(f"  {name:<20} {cnt:>6} boxes")
    print(f"  {'TOTAL':<20} {sum(coarse_counts.values()):>6} boxes")

    print(f"\n  Done — {total_imgs} images → {proc_dir}")
    print(f"  20 fine classes → 7 coarse classes after inference merge")

if __name__ == "__main__":
    from multiprocessing import freeze_support; freeze_support()
    main()
