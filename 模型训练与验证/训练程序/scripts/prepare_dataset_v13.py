#!/usr/bin/env python3
"""V13: 8-class detection (vehicle+road_obstacle merged → obstacle)"""
import argparse, json, os, random, shutil, sys, csv, xml.etree.ElementTree as ET
from collections import defaultdict, Counter
from pathlib import Path
import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
COARSE_IDS = {}
LABEL_NAMES = ['recyclable','kitchen_waste','hazardous','other_waste','green_waste',
               'pedestrian','obstacle','stain']

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

def _coco(ds, m_rules, pd, default_prefix, default_img_subdir=None, cap_key="per_class_cap"):
    p = make_abs(ds["path"]); ap = p / ds["annotation_file"]
    if not ap.exists(): return 0, {}
    with open(ap, encoding="utf-8") as f: coco = json.load(f)
    i2i = {i["id"]: {"fn": i["file_name"], "w": i["width"], "h": i["height"]} for i in coco.get("images", [])}
    i2c = {c["id"]: c["name"] for c in coco.get("categories", [])}
    ia = defaultdict(list); stats = {"t": 0, "k": 0, "d": 0, "pc": defaultdict(int)}
    id2n = {v: k for k, v in COARSE_IDS.items()}
    for ann in coco.get("annotations", []):
        stats["t"] += 1; lbl = resolve_mapping(m_rules, i2c.get(ann["category_id"], ""))
        if lbl is None or lbl == "discard": stats["d"] += 1; continue
        info = i2i.get(ann["image_id"])
        if info is None: stats["d"] += 1; continue
        cx, cy, nw, nh = cb2y(ann["bbox"], info["w"], info["h"])
        if nw <= 0.001 or nh <= 0.001: stats["d"] += 1; continue
        ia[ann["image_id"]].append(f"{COARSE_IDS[lbl]} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")
        stats["k"] += 1; stats["pc"][lbl] += 1
    cap = ds.get(cap_key, None)
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
    img_subdir = ds.get("img_subdir", default_img_subdir)
    prefix = ds.get("prefix", default_prefix)
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
    prefix = ds.get("prefix", "self")
    for jf in sorted(p.glob("*.json")):
        with open(jf, encoding="utf-8") as f: data = json.load(f)
        shapes = data.get("shapes", [])
        img_name = data.get("imagePath", jf.stem + ".jpg"); ip = p / img_name
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
        wm(ip, mi, ml, f"{prefix}_{jf.stem}", lines); imgs += 1
    return imgs, stats

def conv_labelme_folder(ds, m_rules, pd):
    """LabelMe JSON in subfolders (folder name = class hint, polygon → bbox conversion)"""
    p = make_abs(ds["path"])
    if not p.exists(): return 0, {}
    mi = pd / "merged" / "images"; ml = pd / "merged" / "labels"
    stats = {"t": 0, "k": 0, "d": 0, "pc": defaultdict(int)}; imgs = 0
    prefix = ds.get("prefix", "ga")
    for cd in sorted(p.iterdir()):
        if not cd.is_dir() or cd.name.startswith("."): continue
        folder_label = resolve_mapping(m_rules, cd.name)
        for jf in sorted(cd.glob("*.json")):
            with open(jf, encoding="utf-8") as f: data = json.load(f)
            shapes = data.get("shapes", [])
            img_name = data.get("imagePath", jf.stem + ".jpg")
            ip = cd / img_name
            if not ip.exists(): continue
            iw = data.get("imageWidth", 1); ih = data.get("imageHeight", 1)
            lines = []
            for s in shapes:
                lbl = s.get("label", "").strip()
                target = resolve_mapping(m_rules, lbl)
                if target is None or target == "discard":
                    if folder_label and folder_label != "discard":
                        target = folder_label
                    else:
                        stats["d"] += 1; continue
                stats["t"] += 1
                pts = s.get("points", [])
                if len(pts) < 2: stats["d"] += 1; continue
                xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
                xmin, xmax = min(xs), max(xs); ymin, ymax = min(ys), max(ys)
                cx = max(0, min(1, (xmin+xmax)/2/iw)); cy = max(0, min(1, (ymin+ymax)/2/ih))
                nw = max(0.001, min(1, (xmax-xmin)/iw)); nh = max(0.001, min(1, (ymax-ymin)/ih))
                lines.append(f"{COARSE_IDS[target]} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")
                stats["k"] += 1; stats["pc"][target] += 1
            wm(ip, mi, ml, f"{prefix}_{cd.name}_{jf.stem}", lines); imgs += 1
    return imgs, stats

def conv_folder(ds, m_rules, pd):
    """folder_class with box_margin, class_filter, and samples_per_filtered support"""
    p = make_abs(ds["path"])
    if not p.exists(): return 0, {}
    mi = pd / "merged" / "images"; ml = pd / "merged" / "labels"
    stats = {"t": 0, "k": 0, "d": 0, "pc": defaultdict(int)}; imgs = 0
    ncls = ds.get("samples_per_class", None)
    margin = ds.get("box_margin", 1.0)
    class_filter = ds.get("class_filter", None)
    per_filtered = ds.get("samples_per_filtered", {})
    prefix = ds.get("prefix", "gb")
    for cd in sorted(p.iterdir()):
        if not cd.is_dir() or cd.name.startswith("."): continue
        if class_filter and cd.name not in class_filter: continue
        lbl = resolve_mapping(m_rules, cd.name)
        if lbl is None or lbl == "discard": continue
        cid = COARSE_IDS[lbl]
        all_imgs = sorted([f for f in cd.iterdir() if f.suffix.lower() in IMG_EXTS])
        stats["t"] += len(all_imgs)
        limit = per_filtered.get(cd.name, ncls)
        if limit is not None and limit > 0 and len(all_imgs) > limit:
            random.shuffle(all_imgs); all_imgs = all_imgs[:limit]
        for img_file in all_imgs:
            line = f"{cid} 0.5 0.5 {margin:.6f} {margin:.6f}\n"
            wm(img_file, mi, ml, f"{prefix}_{cd.name}_{img_file.stem}", [line])
            stats["k"] += 1; stats["pc"][lbl] += 1; imgs += 1
    return imgs, stats

def conv_csv(ds, m_rules, pd):
    """csv_classify with box_margin support"""
    p = make_abs(ds["path"]); csvp = p / "train.csv"
    if not csvp.exists(): return 0, {}
    mi = pd / "merged" / "images"; ml = pd / "merged" / "labels"
    stats = {"t": 0, "k": 0, "d": 0, "pc": defaultdict(int)}; imgs = 0
    ncls = ds.get("samples_per_class", None)
    margin = ds.get("box_margin", 1.0)
    prefix = ds.get("prefix", "leaf")
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
        line = f"{cid} 0.5 0.5 {margin:.6f} {margin:.6f}\n"
        wm(ip, mi, ml, f"{prefix}_{ip.stem}", [line])
        stats["k"] += 1; stats["pc"][target] += 1; imgs += 1
    return imgs, stats

def conv_yolo_remap(ds, m_rules, pd):
    p = make_abs(ds["path"])
    mi = pd / "merged" / "images"; ml = pd / "merged" / "labels"
    stats = {"t": 0, "k": 0, "d": 0, "pc": defaultdict(int)}; imgs = 0
    prefix = ds.get("prefix", "street")
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
            wm(img_file, mi, ml, f"{prefix}_{split}_{stem}", new_lines); imgs += 1
    return imgs, stats

def conv_voc(ds, m_rules, pd):
    """VOC XML with configurable img_subdir/ann_subdir for non-standard layouts"""
    p = make_abs(ds["path"])
    if not p.exists(): return 0, {}
    mi = pd / "merged" / "images"; ml = pd / "merged" / "labels"
    stats = {"t": 0, "k": 0, "d": 0, "pc": defaultdict(int)}; imgs = 0
    prefix = ds.get("prefix", "voc")

    # Auto-detect layout: VOC2007 (Annotations+JPEGImages), subdir-config, or flat
    ann_subdir = ds.get("ann_subdir", None)
    img_subdir = ds.get("img_subdir", None)

    if ann_subdir:
        ann_dir = p / ann_subdir
    elif (p / "Annotations").exists():
        ann_dir = p / "Annotations"
    elif (p / "annotation").exists():
        ann_dir = p / "annotation"
    else:
        ann_dir = p

    if img_subdir:
        img_dir = p / img_subdir
    elif (p / "JPEGImages").exists():
        img_dir = p / "JPEGImages"
    elif (p / "images").exists():
        img_dir = p / "images"
    elif (p / "Images").exists():
        img_dir = p / "Images"
    else:
        img_dir = p

    for xf in sorted(os.listdir(str(ann_dir))):
        if not xf.endswith(".xml"): continue
        tree = ET.parse(str(ann_dir / xf)); root = tree.getroot()
        fn = root.findtext("filename", ""); ip = img_dir / fn
        if not ip.exists(): continue
        sz = root.find("size"); iw = int(sz.findtext("width", "640")); ih = int(sz.findtext("height", "640"))
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
        wm(ip, mi, ml, f"{prefix}_{Path(fn).stem}", new_lines); imgs += 1
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

def quality_report(pd):
    """完整数据集质检报告 — 8类"""
    names = LABEL_NAMES
    nc = len(names)
    print(f"\n{'='*60}")
    print(f"  V13 PRE-TRAINING QUALITY REPORT ({nc} classes)")
    print(f"{'='*60}")

    # A. 类别分布
    class_counts = defaultdict(int)
    for split in ['train','val','test']:
        ld = pd / split / 'labels'
        if not ld.exists(): continue
        for lf in ld.iterdir():
            with open(lf) as f:
                for line in f:
                    parts = line.strip().split()
                    if parts: class_counts[(split, int(parts[0]))] += 1
    print(f"\n  A. CLASS DISTRIBUTION")
    print(f"  {'Class':<22} {'train':>8} {'val':>8} {'test':>8} {'total':>8} {'status':>10}")
    print(f"  {'-'*22} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*10}")
    for cid in range(nc):
        tr = class_counts.get(('train',cid),0); vl = class_counts.get(('val',cid),0)
        te = class_counts.get(('test',cid),0); total = tr+vl+te
        st = 'OK' if tr>=200 else ('WEAK' if tr>0 else 'MISSING')
        print(f"  {names[cid]:<22} {tr:>8} {vl:>8} {te:>8} {total:>8} {st:>10}")
    tr_total = sum(class_counts.get(('train',c),0) for c in range(nc))
    print(f"  {'TOTAL':<22} {tr_total:>8}")

    # B. 标注质量
    empty = corrupt = dup = invalid = 0; full_img = 0
    for lf in (pd / 'train' / 'labels').iterdir():
        with open(lf) as f: lines = [l.strip() for l in f.readlines() if l.strip()]
        if not lines: empty += 1; continue
        seen = set()
        for line in lines:
            parts = line.split()
            if len(parts) < 5: corrupt += 1; continue
            try:
                cx, cy, nw, nh = float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
            except: corrupt += 1; continue
            if cx<0 or cx>1 or cy<0 or cy>1 or nw<0 or nw>1: invalid += 1
            if abs(cx-0.5)<0.01 and abs(cy-0.5)<0.01 and nw>0.85 and nh>0.85:
                full_img += 1
            key = (int(parts[0]), round(cx,5), round(cy,5), round(nw,5), round(nh,5))
            if key in seen: dup += 1
            seen.add(key)
    print(f"\n  B. ANNOTATION QUALITY (train)")
    print(f"  Empty labels (no boxes):     {empty}")
    print(f"  Corrupt lines:               {corrupt}")
    print(f"  Invalid coordinates:         {invalid}")
    print(f"  Duplicate boxes:             {dup}")
    print(f"  Full-img boxes (margin>85%): {full_img} ({full_img/tr_total*100:.1f}%)")

    # C. 框尺寸分布
    areas = []
    per_class_areas = defaultdict(list)
    for lf in (pd / 'train' / 'labels').iterdir():
        with open(lf) as f:
            for line in f:
                parts = line.strip().split()
                if not parts: continue
                nw, nh = float(parts[3]), float(parts[4])
                areas.append(nw*nh)
                per_class_areas[int(parts[0])].append(nw*nh)
    areas = np.array(areas)
    print(f"\n  C. BOX SCALE DISTRIBUTION (train)")
    for pct in [1,5,10,25,50,75,90,95,99]:
        print(f"  P{pct:>2}: {np.percentile(areas,pct)*100:>6.1f}%")
    tiny = np.sum(areas<0.01)/len(areas)*100
    huge = np.sum(areas>0.9)/len(areas)*100
    print(f"  Tiny (<1%):  {tiny:.1f}%")
    print(f"  Huge (>90%): {huge:.1f}%")

    # D. 各类别框面积中位数
    print(f"\n  D. PER-CLASS BOX AREA MEDIAN")
    for cid in range(nc):
        a = per_class_areas.get(cid, [])
        if a: med = np.median(a)*100; p25 = np.percentile(a,25)*100; p75 = np.percentile(a,75)*100
        else: med = p25 = p75 = 0
        print(f"  {names[cid]:<22} median={med:.1f}%  IQR=[{p25:.1f}%,{p75:.1f}%]")

    # E. 数据源组成
    src_rules = [
        ("taco_", "TACO"), ("dat_", "datas"), ("street_", "street_obstacle"),
        ("pdl_", "puddle_voc"), ("puddle_large_", "puddle_large"), ("self_", "self_collected"),
        ("gb_", "garbages_fullimg"), ("leaf_", "leaf_classify"), ("tvl_", "trainval_voc"),
        ("ga_", "garbages_annotated"), ("car_", "car_identify"), ("person_", "pedestrian_coco"),
        ("traffic_", "traffic_sign"), ("water_", "cb8c9_water"), ("dsr_", "dataset_resized"),
    ]
    src_count = Counter()
    for img_file in (pd / 'train' / 'images').iterdir():
        s = img_file.stem
        src = "unknown"
        for prefix, name in src_rules:
            if s.startswith(prefix):
                src = name; break
        src_count[src] += 1
    print(f"\n  E. DATA SOURCE COMPOSITION (train)")
    for src, cnt in src_count.most_common():
        pct = cnt/sum(src_count.values())*100 if sum(src_count.values())>0 else 0
        print(f"  {src:<25} {cnt:>6} ({pct:.1f}%)")

    # F. 交叉校验
    tr_sizes = set()
    tr_images = list((pd / 'train' / 'images').iterdir())
    for img_file in random.sample(tr_images, min(50, len(tr_images))):
        try:
            from PIL import Image
            img = Image.open(img_file)
            tr_sizes.add(img.size)
        except: pass
    vl_sizes = set()
    vl_dir = pd / 'val' / 'images'
    if vl_dir.exists():
        vl_images = list(vl_dir.iterdir())
        for img_file in random.sample(vl_images, min(50, len(vl_images))):
            try:
                from PIL import Image; img = Image.open(img_file); vl_sizes.add(img.size)
            except: pass
    overlap = tr_sizes & vl_sizes
    print(f"\n  F. TRAIN/VAL SIZE CONSISTENCY")
    print(f"  Train unique sizes (sample 50): {len(tr_sizes)}")
    print(f"  Val unique sizes (sample 50):   {len(vl_sizes)}")
    print(f"  Overlap: {len(overlap)} sizes shared")
    print(f"  Result: {'PASS' if len(overlap) > 2 else 'WARN — possible domain gap'}")
    print(f"\n{'='*60}")

def main():
    global COARSE_IDS
    mapping = load_yaml(PROJECT_ROOT / "configs/label_mapping_v13.yaml")
    COARSE_IDS = {v: k for k, v in mapping["target_classes"].items()}
    sources = load_yaml(PROJECT_ROOT / "configs/source_datasets_v13.yaml")
    pd = PROJECT_ROOT.parent / "数据集" / "processed_v13"
    if pd.exists(): shutil.rmtree(pd)
    (pd / "merged" / "images").mkdir(parents=True); (pd / "merged" / "labels").mkdir(parents=True)

    handlers = {
        "coco_taco": lambda ds, m, pd: _coco(ds, m, pd, "taco", None),
        "coco_flat": lambda ds, m, pd: _coco(ds, m, pd, ds.get("prefix", "dat"),
                                              ds.get("img_subdir", "images"),
                                              "per_class_cap"),
        "yolo_remap": conv_yolo_remap,
        "voc_xml": conv_voc,
        "labelme_json": conv_labelme,
        "labelme_folder": conv_labelme_folder,
        "folder_class": conv_folder,
        "csv_classify": conv_csv,
        "yolo_txt": lambda *a: (0, {}),
    }

    print("=" * 60)
    print("  V13: 8-class — vehicle+road_obstacle merged → obstacle")
    print("=" * 60)
    total_imgs = 0
    for ds in sources["datasets"]:
        name = ds["name"]; fmt = ds["format"]
        if ds.get("enabled") is False:
            print(f"\n[{name}] SKIPPED (disabled)"); continue
        rules = mapping["mappings"].get(ds.get("mapping",""), {})
        h = handlers.get(fmt)
        if not h: print(f"\n[{name}] UNKNOWN fmt={fmt}"); continue
        margin_info = f' margin={ds["box_margin"]}' if ds.get("box_margin") and ds["box_margin"] < 1.0 else ''
        print(f"\n[{name}] format={fmt}{margin_info}")
        n, stats = h(ds, rules, pd); total_imgs += n
        if isinstance(stats, dict) and stats:
            print(f"  Images: {n} | Kept={stats.get('k','?')} / Discarded={stats.get('d','?')} / Total={stats.get('t','?')}")
    print("\n[Splitting] 7:1.5:1.5 ...")
    split_dataset(pd)
    quality_report(pd)
    print(f"\n  Done — {total_imgs} images → {pd}")

if __name__ == "__main__":
    from multiprocessing import freeze_support; freeze_support()
    main()
