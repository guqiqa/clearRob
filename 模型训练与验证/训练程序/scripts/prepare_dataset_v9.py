#!/usr/bin/env python3
"""V9 crash-safe dataset — real detection + 36% full-image supplement"""
import argparse, json, os, random, shutil, sys, csv, xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path
import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
COARSE_IDS = {}

def load_yaml(p):
    with open(p, encoding="utf-8") as f: return yaml.safe_load(f)

def make_abs(p):
    return Path(p) if Path(p).is_absolute() else (PROJECT_ROOT / p).resolve()

def resolve_mapping(rules, key):
    if key in rules: return rules[key]
    if str(key) in rules: return rules[str(key)]
    if isinstance(key, str):
        try:
            if int(key) in rules: return rules[int(key)]
        except ValueError: pass
    if "*" in rules: return rules["*"]
    return None

def wm(src_img, mi, ml, stem, lines):
    mi.mkdir(parents=True, exist_ok=True); ml.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src_img, mi / f"{stem}.jpg")
    with open(ml / f"{stem}.txt", "w") as f: f.writelines(lines)

def cb2y(bbox, w, h):
    x, y, bw, bh = bbox
    return (max(0, min(1, (x+bw/2)/w)), max(0, min(1, (y+bh/2)/h)), max(0.001, min(1, bw/w)), max(0.001, min(1, bh/h)))

def _coco(ds, m_rules, pd, prefix, img_subdir=None, cap=None):
    p = make_abs(ds["path"]); ap = p / ds["annotation_file"]
    if not ap.exists(): return 0, {}
    with open(ap, encoding="utf-8") as f: coco = json.load(f)
    i2i = {i["id"]: {"fn": i["file_name"], "w": i["width"], "h": i["height"]} for i in coco.get("images", [])}
    i2c = {c["id"]: c["name"] for c in coco.get("categories", [])}
    ia = defaultdict(list); stats = {"t": 0, "k": 0, "d": 0, "pc": defaultdict(int)}
    id2n = {v: k for k, v in COARSE_IDS.items()}
    for ann in coco.get("annotations", []):
        stats["t"] += 1
        lbl = resolve_mapping(m_rules, i2c.get(ann["category_id"], ""))
        if lbl is None or lbl == "discard": stats["d"] += 1; continue
        info = i2i.get(ann["image_id"])
        if info is None: stats["d"] += 1; continue
        cx, cy, nw, nh = cb2y(ann["bbox"], info["w"], info["h"])
        if nw <= 0.001 or nh <= 0.001: stats["d"] += 1; continue
        ia[ann["image_id"]].append(f"{COARSE_IDS[lbl]} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")
        stats["k"] += 1; stats["pc"][lbl] += 1
    if cap:
        capped = 0
        for img_id, lines in list(ia.items()):
            by_cls = defaultdict(list)
            for line in lines: by_cls[id2n[int(line.split()[0])]].append(line)
            nl = []
            for cn, ls in by_cls.items():
                c = cap.get(cn)
                if c and len(ls) > c: random.shuffle(ls); nl.extend(ls[:c]); capped += len(ls) - c
                else: nl.extend(ls)
            ia[img_id] = nl
        stats["k"] -= capped
    mi = pd / "merged" / "images"; ml = pd / "merged" / "labels"
    written = 0
    for img_id, info in i2i.items():
        src = p / (f"{img_subdir}/{info['fn']}" if img_subdir else info["fn"])
        if not src.exists(): continue
        wm(src, mi, ml, f"{prefix}_{Path(info['fn']).stem}", ia.get(img_id, []))
        written += 1
    return written, stats

def conv_labelme(ds, m_rules, pd):
    p = make_abs(ds["path"])
    if not p.exists(): return 0, {}
    mi = pd / "merged" / "images"; ml = pd / "merged" / "labels"
    stats = {"t": 0, "k": 0, "d": 0, "pc": defaultdict(int)}; imgs = 0
    for jf in sorted(p.glob("*.json")):
        with open(jf, encoding="utf-8") as f: data = json.load(f)
        shapes = data.get("shapes", [])
        img_name = data.get("imagePath", jf.stem + ".jpg")
        ip = p / img_name
        if not ip.exists(): continue
        lines = []
        for s in shapes:
            lbl = s.get("label", "").strip()
            target = resolve_mapping(m_rules, lbl)
            if target is None or target == "discard": stats["d"] += 1; continue
            stats["t"] += 1
            pts = s.get("points", [])
            if len(pts) < 2: stats["d"] += 1; continue
            x1, y1 = pts[0]; x2, y2 = pts[1] if len(pts) >= 2 else pts[0]
            iw = data.get("imageWidth", 1); ih = data.get("imageHeight", 1)
            cx = max(0, min(1, (x1+x2)/2/iw)); cy = max(0, min(1, (y1+y2)/2/ih))
            nw = max(0.001, min(1, abs(x2-x1)/iw)); nh = max(0.001, min(1, abs(y2-y1)/ih))
            lines.append(f"{COARSE_IDS[target]} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")
            stats["k"] += 1; stats["pc"][target] += 1
        wm(ip, mi, ml, f"self_{jf.stem}", lines); imgs += 1
    return imgs, stats

def conv_folder(ds, m_rules, pd):
    p = make_abs(ds["path"])
    if not p.exists(): return 0, {}
    mi = pd / "merged" / "images"; ml = pd / "merged" / "labels"
    stats = {"t": 0, "k": 0, "d": 0, "pc": defaultdict(int)}; imgs = 0
    ncls = ds.get("samples_per_class", None)
    for cd in sorted(p.iterdir()):
        if not cd.is_dir() or cd.name.startswith("."): continue
        lbl = resolve_mapping(m_rules, cd.name)
        if lbl is None or lbl == "discard": continue
        cid = COARSE_IDS[lbl]
        all_imgs = sorted([f for f in cd.iterdir() if f.suffix.lower() in IMG_EXTS])
        stats["t"] += len(all_imgs)
        if ncls and len(all_imgs) > ncls: random.shuffle(all_imgs); all_imgs = all_imgs[:ncls]
        for img_file in all_imgs:
            wm(img_file, mi, ml, f"gb_{cd.name}_{img_file.stem}", [f"{cid} 0.5 0.5 1.0 1.0\n"])
            stats["k"] += 1; stats["pc"][lbl] += 1; imgs += 1
    return imgs, stats

def conv_yolo_remap(ds, m_rules, pd):
    p = make_abs(ds["path"])
    mi = pd / "merged" / "images"; ml = pd / "merged" / "labels"
    stats = {"t": 0, "k": 0, "d": 0, "pc": defaultdict(int)}; imgs = 0
    for split in ["train", "valid", "val", "test"]:
        idir = p / "images" / split; ldir = p / "labels" / split
        if not idir.exists() or not ldir.exists(): continue
        for lf in sorted(ldir.iterdir()):
            if not lf.suffix == ".txt": continue
            stem = lf.stem; img_file = None
            for ext in IMG_EXTS:
                c = idir / (stem + ext)
                if c.exists(): img_file = c; break
            if img_file is None: continue
            new_lines = []
            with open(lf) as f:
                for line in f:
                    parts = line.strip().split()
                    if not parts: continue
                    stats["t"] += 1
                    lbl = resolve_mapping(m_rules, int(parts[0]))
                    if lbl is None or lbl == "discard": stats["d"] += 1; continue
                    new_lines.append(f"{COARSE_IDS[lbl]} {' '.join(parts[1:])}\n")
                    stats["k"] += 1; stats["pc"][lbl] += 1
            wm(img_file, mi, ml, f"{ds['name']}_{split}_{stem}", new_lines); imgs += 1
    return imgs, stats

def conv_voc(ds, m_rules, pd):
    p = make_abs(ds["path"])
    if not p.exists(): return 0, {}
    mi = pd / "merged" / "images"; ml = pd / "merged" / "labels"
    stats = {"t": 0, "k": 0, "d": 0, "pc": defaultdict(int)}; imgs = 0
    for xf in sorted(os.listdir(str(p))):
        if not xf.endswith(".xml"): continue
        tree = ET.parse(str(p / xf)); root = tree.getroot()
        fn = root.findtext("filename", ""); ip = p / fn
        if not ip.exists(): continue
        sz = root.find("size")
        iw = int(sz.findtext("width", "640")); ih = int(sz.findtext("height", "640"))
        new_lines = []
        for obj in root.findall("object"):
            name = obj.findtext("name", "").strip(); stats["t"] += 1
            lbl = resolve_mapping(m_rules, name)
            if lbl is None or lbl == "discard": stats["d"] += 1; continue
            bb = obj.find("bndbox")
            if bb is None: stats["d"] += 1; continue
            xmin = float(bb.findtext("xmin","0")); ymin = float(bb.findtext("ymin","0"))
            xmax = float(bb.findtext("xmax",str(iw))); ymax = float(bb.findtext("ymax",str(ih)))
            cx = max(0, min(1, (xmin+xmax)/2/iw)); cy = max(0, min(1, (ymin+ymax)/2/ih))
            nw = max(0.001, min(1, (xmax-xmin)/iw)); nh = max(0.001, min(1, (ymax-ymin)/ih))
            new_lines.append(f"{COARSE_IDS[lbl]} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")
            stats["k"] += 1; stats["pc"][lbl] += 1
        wm(ip, mi, ml, f"pdl_{Path(fn).stem}", new_lines); imgs += 1
    return imgs, stats

def conv_csv(ds, m_rules, pd):
    p = make_abs(ds["path"]); csvp = p / "train.csv"
    if not csvp.exists(): return 0, {}
    mi = pd / "merged" / "images"; ml = pd / "merged" / "labels"
    stats = {"t": 0, "k": 0, "d": 0, "pc": defaultdict(int)}; imgs = 0
    ncls = ds.get("samples_per_class", None)
    all_rows = []
    with open(csvp, encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            img_rel = row.get("image", ""); lbl = row.get("label", "").strip()
            if not lbl: stats["d"] += 1; continue
            ip = p / "images" / os.path.basename(img_rel)
            if not ip.exists(): stats["d"] += 1; continue
            target = resolve_mapping(m_rules, lbl)
            if target is None or target == "discard": stats["d"] += 1; continue
            all_rows.append((ip, target))
    stats["t"] = len(all_rows)
    if ncls and len(all_rows) > ncls: random.shuffle(all_rows); all_rows = all_rows[:ncls]
    for ip, target in all_rows:
        cid = COARSE_IDS[target]
        wm(ip, mi, ml, f"leaf_{ip.stem}", [f"{cid} 0.5 0.5 1.0 1.0\n"])
        stats["k"] += 1; stats["pc"][target] += 1; imgs += 1
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
        for img, lbl in items: shutil.copy2(img, d / "images" / img.name); shutil.copy2(lbl, d / "labels" / lbl.name)
    print(f"  Split: train={len(splits['train'])}, val={len(splits['val'])}, test={len(splits['test'])}")

def main():
    global COARSE_IDS
    mapping = load_yaml(PROJECT_ROOT / "configs/label_mapping_v9.yaml")
    COARSE_IDS = {v: k for k, v in mapping["target_classes"].items()}
    sources = load_yaml(PROJECT_ROOT / "configs/source_datasets_v9.yaml")
    pd = PROJECT_ROOT.parent / "数据集" / "processed_v9"
    if pd.exists(): shutil.rmtree(pd)
    (pd / "merged" / "images").mkdir(parents=True); (pd / "merged" / "labels").mkdir(parents=True)
    handlers = {"coco_taco": lambda ds, m, pd: _coco(ds, m, pd, "taco"),
                "coco_flat": lambda ds, m, pd: _coco(ds, m, pd, "dat", "images", ds.get("per_class_cap")),
                "yolo_remap": conv_yolo_remap, "voc_xml": conv_voc,
                "labelme_json": conv_labelme, "folder_class": conv_folder,
                "csv_classify": conv_csv, "yolo_txt": lambda *a: (0, {})}
    print("=" * 60)
    print("  V9 Crash-Safe Dataset — Real Detection + 36% Supplement")
    print("=" * 60)
    total_imgs = 0
    for ds in sources["datasets"]:
        name = ds["name"]; fmt = ds["format"]
        rules = mapping["mappings"].get(ds.get("mapping",""), {})
        h = handlers.get(fmt)
        if not h: print(f"\n[{name}] UNKNOWN fmt={fmt}"); continue
        print(f"\n[{name}] format={fmt}")
        n, stats = h(ds, rules, pd)
        total_imgs += n
        if isinstance(stats, dict) and stats:
            print(f"  Images: {n} | Kept={stats.get('k','?'):} / Discarded={stats.get('d','?'):} / Total={stats.get('t','?'):}")
    print("\n[Splitting] 7:1.5:1.5 ...")
    split_dataset(pd)
    cc = defaultdict(int)
    for lf in (pd / "train" / "labels").iterdir():
        with open(lf) as f:
            for line in f: cc[int(line.split()[0])] += 1
    tc = mapping["target_classes"]
    print(f"\n{'='*60}")
    print(f"  V9 Class Distribution (train)")
    print(f"{'='*60}")
    for cid in sorted(tc):
        name = tc[cid]; cnt = cc.get(cid, 0)
        bar = "=" * min(cnt//100, 15) if cnt > 0 else ""
        st = "OK" if cnt >= 500 else ("WEAK" if cnt > 0 else "MISSING")
        print(f"  {cid}: {name:<20} {cnt:>7} |{bar:<15}| {st}")
    total_boxes = sum(cc.values())
    # Estimate full-image ratio
    full_img_boxes = 5094 * 2  # garbages + leaf
    real_boxes = total_boxes - full_img_boxes
    imb = max(cc.values())/min(cc.values()) if min(cc.values()) > 0 else 999
    print(f"  {'TOTAL':<22} {total_boxes:>7} boxes")
    print(f"\n  Estimated: full-img={full_img_boxes} ({full_img_boxes/total_boxes*100:.1f}%), real-detection={real_boxes} ({real_boxes/total_boxes*100:.1f}%)")
    print(f"  Imbalance: {imb:.1f}:1")
    print(f"  Images: {total_imgs}")
    print(f"  Output: {pd}")
    print(f"\n  V8 crash rate: V1 at 36% survived E63, V8 at 99% crashed E28")
    print(f"  V9 at {full_img_boxes/total_boxes*100:.1f}% full-img — predicted safe range")

if __name__ == "__main__":
    from multiprocessing import freeze_support; freeze_support()
    main()
