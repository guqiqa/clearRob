#!/usr/bin/env python3
"""
V8 数据集准备 — 全部数据集（排除 TACO）→ 7 粗类 YOLO txt
"""

import argparse, json, os, random, shutil, sys, csv, xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
COARSE_IDS = {}  # coarse class name → ID


def load_yaml(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def resolve_mapping(rules, key):
    if key in rules: return rules[key]
    if str(key) in rules: return rules[str(key)]
    if isinstance(key, str):
        try:
            if int(key) in rules: return rules[int(key)]
        except ValueError: pass
    if "*" in rules: return rules["*"]
    return None


def write_merged(src_img, merged_img, merged_lbl, stem, lines):
    merged_img.mkdir(parents=True, exist_ok=True)
    merged_lbl.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src_img, merged_img / f"{stem}.jpg")
    if lines:
        with open(merged_lbl / f"{stem}.txt", "w") as f:
            f.writelines(lines)


def coco_bbox_to_yolo(bbox, w, h):
    x, y, bw, bh = bbox
    return (max(0, min(1, (x+bw/2)/w)), max(0, min(1, (y+bh/2)/h)),
            max(0, min(1, bw/w)), max(0, min(1, bh/h)))


def _coco_core(ds, mapping_rules, proc_dir, prefix, img_subdir=None, per_class_cap=None):
    p = make_abs(ds["path"])
    ap = p / ds["annotation_file"]
    if not ap.exists(): return 0, {}
    with open(ap, encoding="utf-8") as f:
        coco = json.load(f)
    id_to_img = {i["id"]: {"fn": i["file_name"], "w": i["width"], "h": i["height"]}
                 for i in coco.get("images", [])}
    id_to_cat = {c["id"]: c["name"] for c in coco.get("categories", [])}
    img_anns = defaultdict(list)
    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    id_to_name = {v: k for k, v in COARSE_IDS.items()}

    for ann in coco.get("annotations", []):
        stats["total"] += 1
        lbl = resolve_mapping(mapping_rules, id_to_cat.get(ann["category_id"], ""))
        if lbl is None or lbl == "discard": stats["discarded"] += 1; continue
        info = id_to_img.get(ann["image_id"])
        if info is None: stats["discarded"] += 1; continue
        cx, cy, nw, nh = coco_bbox_to_yolo(ann["bbox"], info["w"], info["h"])
        if nw <= 0.001 or nh <= 0.001: stats["discarded"] += 1; continue
        cid = COARSE_IDS[lbl]
        img_anns[ann["image_id"]].append(f"{cid} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")
        stats["kept"] += 1; stats["per_class"][lbl] += 1

    if per_class_cap:
        capped = 0
        for img_id, lines in list(img_anns.items()):
            by_cls = defaultdict(list)
            for line in lines:
                by_cls[id_to_name[int(line.split()[0])]].append(line)
            new_lines = []
            for cn, ls in by_cls.items():
                cap = per_class_cap.get(cn)
                if cap and len(ls) > cap:
                    random.shuffle(ls); new_lines.extend(ls[:cap]); capped += len(ls) - cap
                else:
                    new_lines.extend(ls)
            img_anns[img_id] = new_lines
        stats["kept"] -= capped

    mi = proc_dir / "merged" / "images"; ml = proc_dir / "merged" / "labels"
    written = 0
    for img_id, info in id_to_img.items():
        src = p / (f"{img_subdir}/{info['fn']}" if img_subdir else info["fn"])
        if not src.exists(): continue
        write_merged(src, mi, ml, f"{prefix}_{Path(info['fn']).stem}", img_anns.get(img_id, []))
        written += 1
    return written, stats


def make_abs(p):
    return Path(p) if Path(p).is_absolute() else (PROJECT_ROOT / p).resolve()


def convert_coco_flat(ds, m, pd):
    return _coco_core(ds, m, pd, "dat", "images", ds.get("per_class_cap"))


def convert_folder_class(ds, m, pd):
    p = make_abs(ds["path"])
    if not p.exists(): return 0, {}
    mi = pd / "merged" / "images"; ml = pd / "merged" / "labels"
    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    imgs = 0
    max_per = ds.get("samples_per_class", None)

    for cd in sorted(p.iterdir()):
        if not cd.is_dir() or cd.name.startswith(".") or cd.name == "__MACOSX": continue
        lbl = resolve_mapping(m, cd.name)
        if lbl is None or lbl == "discard": continue
        cid = COARSE_IDS[lbl]
        all_imgs = sorted([f for f in cd.iterdir() if f.suffix.lower() in IMG_EXTS])
        stats["total"] += len(all_imgs)
        if max_per and len(all_imgs) > max_per:
            random.shuffle(all_imgs); all_imgs = all_imgs[:max_per]
        for img_file in all_imgs:
            line = f"{cid} 0.5 0.5 1.0 1.0\n"
            write_merged(img_file, mi, ml, f"gb_{cd.name}_{img_file.stem}", [line])
            stats["kept"] += 1; stats["per_class"][lbl] += 1; imgs += 1
    return imgs, stats


def convert_folder_40class(ds, m, pd):
    """train/ dataset: numeric subdirs (0-39), 40-class garbage classification"""
    p = make_abs(ds["path"]) / "images"
    if not p.exists(): return 0, {}
    mi = pd / "merged" / "images"; ml = pd / "merged" / "labels"
    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    imgs = 0

    for cd in sorted(p.iterdir()):
        if not cd.is_dir(): continue
        lbl = resolve_mapping(m, cd.name)
        if lbl is None or lbl == "discard": continue
        cid = COARSE_IDS[lbl]
        all_imgs = [f for f in cd.iterdir() if f.suffix.lower() in IMG_EXTS]
        stats["total"] += len(all_imgs)
        for img_file in all_imgs:
            line = f"{cid} 0.5 0.5 1.0 1.0\n"
            write_merged(img_file, mi, ml, f"trn_{cd.name}_{img_file.stem}", [line])
            stats["kept"] += 1; stats["per_class"][lbl] += 1; imgs += 1
    return imgs, stats


def convert_classify_csv(ds, m, pd):
    """garbage_classify: train_data/*.txt contains 'filename.jpg, class_id'"""
    p = make_abs(ds["path"]) / "train_data"
    if not p.exists(): return 0, {}
    mi = pd / "merged" / "images"; ml = pd / "merged" / "labels"
    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    imgs = 0

    for tf in sorted(p.iterdir()):
        if not tf.suffix == ".txt": continue
        with open(tf, encoding="utf-8") as f:
            content = f.read().strip()
        if not content: continue
        if "," in content:
            img_name, cls_str = content.rsplit(",", 1)
        else:
            parts = content.rsplit(" ", 1); img_name, cls_str = parts[0], parts[1]
        img_name = img_name.strip(); cls_str = cls_str.strip()
        ip = p / img_name
        if not ip.exists(): continue
        stats["total"] += 1
        lbl = resolve_mapping(m, int(cls_str))
        if lbl is None or lbl == "discard": stats["discarded"] += 1; continue
        cid = COARSE_IDS[lbl]
        line = f"{cid} 0.5 0.5 1.0 1.0\n"
        write_merged(ip, mi, ml, f"gc_{Path(img_name).stem}", [line])
        stats["kept"] += 1; stats["per_class"][lbl] += 1; imgs += 1
    return imgs, stats


def convert_csv_classify(ds, m, pd):
    """leaf_classify: train.csv with 'image, label'"""
    p = make_abs(ds["path"])
    csv_path = p / "train.csv"
    if not csv_path.exists(): return 0, {}
    mi = pd / "merged" / "images"; ml = pd / "merged" / "labels"
    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    imgs = 0

    with open(csv_path, encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            img_rel = row.get("image", ""); lbl = row.get("label", "").strip()
            if not lbl: stats["discarded"] += 1; continue
            ip = p / "images" / os.path.basename(img_rel)
            if not ip.exists(): stats["discarded"] += 1; continue
            stats["total"] += 1
            target = resolve_mapping(m, lbl)
            if target is None or target == "discard": stats["discarded"] += 1; continue
            cid = COARSE_IDS[target]
            write_merged(ip, mi, ml, f"leaf_{Path(img_rel).stem}", [f"{cid} 0.5 0.5 1.0 1.0\n"])
            stats["kept"] += 1; stats["per_class"][target] += 1; imgs += 1
    return imgs, stats


def convert_yolo_remap(ds, m, pd):
    p = make_abs(ds["path"])
    mi = pd / "merged" / "images"; ml = pd / "merged" / "labels"
    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    imgs = 0

    for split in ["train", "valid", "val", "test"]:
        idir = p / "images" / split; ldir = p / "labels" / split
        if not idir.exists() or not ldir.exists(): continue
        for lf in sorted(ldir.iterdir()):
            if not lf.suffix == ".txt": continue
            stem = lf.stem
            img_file = None
            for ext in IMG_EXTS:
                c = idir / (stem + ext)
                if c.exists(): img_file = c; break
            if img_file is None: continue
            new_lines = []
            with open(lf) as f:
                for line in f:
                    parts = line.strip().split()
                    if not parts: continue
                    stats["total"] += 1
                    lbl = resolve_mapping(m, int(parts[0]))
                    if lbl is None or lbl == "discard": stats["discarded"] += 1; continue
                    cid = COARSE_IDS[lbl]
                    new_lines.append(f"{cid} {' '.join(parts[1:])}\n")
                    stats["kept"] += 1; stats["per_class"][lbl] += 1
            write_merged(img_file, mi, ml, f"{ds['name']}_{split}_{stem}", new_lines)
            imgs += 1
    return imgs, stats


def convert_voc_xml(ds, m, pd):
    p = make_abs(ds["path"])
    if not p.exists(): return 0, {}
    mi = pd / "merged" / "images"; ml = pd / "merged" / "labels"
    stats = {"total": 0, "kept": 0, "discarded": 0, "per_class": defaultdict(int)}
    imgs = 0

    for xf in sorted(os.listdir(str(p))):
        if not xf.endswith(".xml"): continue
        tree = ET.parse(str(p / xf)); root = tree.getroot()
        fn = root.findtext("filename", "")
        ip = p / fn
        if not ip.exists(): continue
        sz = root.find("size")
        iw = int(sz.findtext("width", "640")); ih = int(sz.findtext("height", "640"))
        new_lines = []
        for obj in root.findall("object"):
            name = obj.findtext("name", "").strip()
            stats["total"] += 1
            lbl = resolve_mapping(m, name)
            if lbl is None or lbl == "discard": stats["discarded"] += 1; continue
            bb = obj.find("bndbox")
            if bb is None: stats["discarded"] += 1; continue
            xmin = float(bb.findtext("xmin", "0")); ymin = float(bb.findtext("ymin", "0"))
            xmax = float(bb.findtext("xmax", str(iw))); ymax = float(bb.findtext("ymax", str(ih)))
            cx = max(0, min(1, (xmin+xmax)/2/iw)); cy = max(0, min(1, (ymin+ymax)/2/ih))
            nw = max(0.001, min(1, (xmax-xmin)/iw)); nh = max(0.001, min(1, (ymax-ymin)/ih))
            cid = COARSE_IDS[lbl]
            new_lines.append(f"{cid} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")
            stats["kept"] += 1; stats["per_class"][lbl] += 1
        write_merged(ip, mi, ml, f"pdl_{Path(fn).stem}", new_lines)
        imgs += 1
    return imgs, stats


def split_dataset(proc_dir):
    mi = proc_dir / "merged" / "images"; ml = proc_dir / "merged" / "labels"
    if not mi.exists(): return
    pairs = []
    for img_file in sorted(mi.iterdir()):
        if img_file.suffix.lower() not in IMG_EXTS: continue
        lf = ml / f"{img_file.stem}.txt"
        if lf.exists(): pairs.append((img_file, lf))
    random.seed(42); random.shuffle(pairs)
    n = len(pairs)
    splits = {"train": pairs[:int(n*0.7)], "val": pairs[int(n*0.7):int(n*0.85)], "test": pairs[int(n*0.85):]}
    for sn, items in splits.items():
        d = proc_dir / sn
        if d.exists(): shutil.rmtree(d)
        (d / "images").mkdir(parents=True); (d / "labels").mkdir(parents=True)
        for img, lbl in items:
            shutil.copy2(img, d / "images" / img.name)
            shutil.copy2(lbl, d / "labels" / lbl.name)
    print(f"  Split: train={len(splits['train'])}, val={len(splits['val'])}, test={len(splits['test'])}")


def main():
    global COARSE_IDS
    mapping = load_yaml(PROJECT_ROOT / "configs/label_mapping_v8.yaml")
    COARSE_IDS = {v: k for k, v in mapping["target_classes"].items()}

    sources = load_yaml(PROJECT_ROOT / "configs/source_datasets_v8.yaml")
    proc_dir = PROJECT_ROOT.parent / "数据集" / "processed_v8"

    if proc_dir.exists(): shutil.rmtree(proc_dir)
    (proc_dir / "merged" / "images").mkdir(parents=True)
    (proc_dir / "merged" / "labels").mkdir(parents=True)

    handlers = {
        "folder_class": convert_folder_class,
        "folder_40class": convert_folder_40class,
        "classify_csv": convert_classify_csv,
        "csv_classify": convert_csv_classify,
        "coco_flat": convert_coco_flat,
        "yolo_remap": convert_yolo_remap,
        "voc_xml": convert_voc_xml,
        "yolo_txt": lambda *a: (0, {}),
    }

    print("=" * 60)
    print("  V8 Dataset Preparation — ALL sources (except TACO)")
    print("=" * 60)

    total_imgs = 0
    for ds in sources["datasets"]:
        name = ds["name"]
        fmt = ds["format"]
        rules = mapping["mappings"].get(ds.get("mapping", ""), {})
        print(f"\n[{name}] format={fmt}")
        handler = handlers.get(fmt)
        if not handler:
            print(f"  UNKNOWN format"); continue
        n, stats = handler(ds, rules, proc_dir)
        total_imgs += n
        if isinstance(stats, dict) and stats:
            print(f"  Images: {n} | Kept={stats['kept']} / Discarded={stats['discarded']} / Total={stats['total']}")

    print("\n[Splitting] 7:1.5:1.5 ...")
    split_dataset(proc_dir)

    # Per-class stats
    class_counts = defaultdict(int)
    for lf in (proc_dir / "train" / "labels").iterdir():
        with open(lf) as f:
            for line in f:
                class_counts[int(line.split()[0])] += 1

    tc = mapping["target_classes"]
    print(f"\n{'='*55}")
    print(f"  Class Distribution (train)")
    print(f"{'='*55}")
    for cid in sorted(tc):
        name = tc[cid]; cnt = class_counts.get(cid, 0)
        bar = "=" * min(cnt // 100, 15) if cnt > 0 else ""
        st = "OK" if cnt >= 500 else ("WEAK" if cnt > 0 else "MISSING")
        print(f"  {cid}: {name:<20} {cnt:>7} |{bar:<15}| {st}")
    print(f"  {'TOTAL':<22} {sum(class_counts.values()):>7} boxes")

    print(f"\n  Done — {total_imgs} images → {proc_dir}")


if __name__ == "__main__":
    from multiprocessing import freeze_support; freeze_support()
    main()
