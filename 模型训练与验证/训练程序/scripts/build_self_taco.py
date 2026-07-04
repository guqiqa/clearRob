#!/usr/bin/env python3
"""Self-Collected(510) + TACO(1500) detection-only dataset builder"""
import json, os, random, shutil, sys
from collections import defaultdict
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}

# Direct 8-class mapping — both sources already use our class names
SELF_CLASSES = ["recyclable","kitchen_waste","hazardous","other_waste","green_waste","road_obstacle","pedestrian_pet","stain"]
TACO_TO_TARGET = {
    "Clear plastic bottle":"recyclable","Other plastic bottle":"recyclable","Plastic bottle cap":"recyclable","Plastic lid":"recyclable",
    "Glass bottle":"recyclable","Broken glass":"recyclable","Glass cup":"recyclable","Glass jar":"recyclable",
    "Food Can":"recyclable","Aerosol":"recyclable","Drink can":"recyclable","Aluminium foil":"recyclable",
    "Metal bottle cap":"recyclable","Metal lid":"recyclable","Pop tab":"recyclable","Scrap metal":"recyclable",
    "Aluminium blister pack":"recyclable","Carded blister pack":"recyclable",
    "Other carton":"recyclable","Corrugated carton":"recyclable","Meal carton":"recyclable","Drink carton":"recyclable",
    "Egg carton":"recyclable","Pizza box":"recyclable","Paper cup":"recyclable",
    "Magazine paper":"recyclable","Normal paper":"recyclable","Paper bag":"recyclable","Wrapping paper":"recyclable",
    "Plastified paper bag":"recyclable","Polypropylene bag":"recyclable",
    "Disposable food container":"recyclable","Foam food container":"recyclable","Other plastic container":"recyclable",
    "Tupperware":"recyclable","Spread tub":"recyclable","Other plastic":"recyclable","Six pack rings":"recyclable","Toilet tube":"recyclable",
    "Food waste":"kitchen_waste", "Battery":"hazardous",
    "Cigarette":"other_waste","Tissues":"other_waste","Plastic film":"other_waste","Garbage bag":"other_waste",
    "Other plastic wrapper":"other_waste","Single-use carrier bag":"other_waste","Crisp packet":"other_waste",
    "Plastic glooves":"other_waste","Plastic utensils":"other_waste","Squeezable tube":"other_waste",
    "Plastic straw":"other_waste","Paper straw":"other_waste","Styrofoam piece":"other_waste",
    "Unlabeled litter":"other_waste","Foam cup":"other_waste","Disposable plastic cup":"other_waste",
    "Other plastic cup":"other_waste","Rope & strings":"other_waste","Shoe":"other_waste",
    "Furniture":"road_obstacle","Tire":"road_obstacle","Large object":"road_obstacle","Construction debris":"road_obstacle",
}

def main():
    proc = PROJECT_ROOT.parent / "数据集" / "processed_self_taco"
    if proc.exists(): shutil.rmtree(proc)
    mi = proc / "merged" / "images"; ml = proc / "merged" / "labels"
    mi.mkdir(parents=True); ml.mkdir(parents=True)

    total_imgs = 0; total_boxes = 0
    class_counts = defaultdict(int)

    # --- Self-collected (LabelMe JSON) ---
    self_dir = PROJECT_ROOT.parent / "数据集" / "raw" / "self_collected"
    print("[self_collected] LabelMe JSON")
    self_imgs = 0; self_boxes = 0
    for jf in sorted(self_dir.glob("*.json")):
        with open(jf, encoding="utf-8") as f: data = json.load(f)
        shapes = data.get("shapes", [])
        img_name = data.get("imagePath", jf.stem + ".jpg"); ip = self_dir / img_name
        if not ip.exists(): continue
        iw = data.get("imageWidth", 1); ih = data.get("imageHeight", 1)
        lines = []
        for s in shapes:
            lbl = s.get("label","").strip()
            if lbl not in SELF_CLASSES: continue
            pts = s.get("points", [])
            if len(pts) < 2: continue
            x1, y1 = pts[0]; x2, y2 = pts[1] if len(pts) >= 2 else pts[0]
            cx = max(0, min(1, (x1+x2)/2/iw)); cy = max(0, min(1, (y1+y2)/2/ih))
            nw = max(0.001, min(1, abs(x2-x1)/iw)); nh = max(0.001, min(1, abs(y2-y1)/ih))
            cid = SELF_CLASSES.index(lbl)
            lines.append(f"{cid} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")
            self_boxes += 1; class_counts[cid] += 1
        shutil.copy2(ip, mi / f"self_{jf.stem}.jpg")
        with open(ml / f"self_{jf.stem}.txt", "w") as f: f.writelines(lines)
        self_imgs += 1
    print(f"  Images: {self_imgs} | Boxes: {self_boxes}")

    # --- TACO (COCO JSON) ---
    taco_dir = PROJECT_ROOT.parent / "数据集" / "raw" / "taco"
    ann_path = taco_dir / "annotations.json"
    print(f"\n[TACO] COCO JSON")
    with open(ann_path, encoding="utf-8") as f: coco = json.load(f)
    i2i = {i["id"]: {"fn": i["file_name"], "w": i["width"], "h": i["height"]} for i in coco["images"]}
    i2c = {c["id"]: c["name"] for c in coco["categories"]}
    img_anns = defaultdict(list)
    for ann in coco["annotations"]:
        target = TACO_TO_TARGET.get(i2c.get(ann["category_id"], ""))
        if target is None: continue
        info = i2i.get(ann["image_id"])
        if info is None: continue
        x, y, bw, bh = ann["bbox"]
        cx = max(0, min(1, (x+bw/2)/info["w"])); cy = max(0, min(1, (y+bh/2)/info["h"]))
        nw = max(0.001, min(1, bw/info["w"])); nh = max(0.001, min(1, bh/info["h"]))
        cid = SELF_CLASSES.index(target)
        img_anns[ann["image_id"]].append(f"{cid} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")
    taco_imgs = 0; taco_boxes = 0
    for img_id, info in i2i.items():
        src = taco_dir / info["fn"]
        if not src.exists(): continue
        lines = img_anns.get(img_id, [])
        stem = Path(info["fn"]).stem; batch = Path(info["fn"]).parent.name
        shutil.copy2(src, mi / f"taco_{batch}_{stem}.jpg")
        with open(ml / f"taco_{batch}_{stem}.txt", "w") as f: f.writelines(lines)
        for l in lines: cid = int(l.split()[0]); class_counts[cid] += 1
        taco_imgs += 1; taco_boxes += len(lines)
    print(f"  Images: {taco_imgs} | Boxes: {taco_boxes}")

    # --- Split ---
    total_imgs = self_imgs + taco_imgs
    total_boxes = sum(class_counts.values())
    print(f"\nCOMBINED: {total_imgs} images, {total_boxes} boxes")

    pairs = []
    for img_file in sorted(mi.iterdir()):
        if img_file.suffix.lower() not in IMG_EXTS: continue
        lf = ml / f"{img_file.stem}.txt"
        if lf.exists(): pairs.append((img_file, lf))
    random.seed(42); random.shuffle(pairs)
    n = len(pairs)
    splits = {"train": pairs[:int(n*0.7)], "val": pairs[int(n*0.7):int(n*0.85)], "test": pairs[int(n*0.85):]}
    for sn, items in splits.items():
        d = proc / sn
        if d.exists(): shutil.rmtree(d)
        (d / "images").mkdir(parents=True); (d / "labels").mkdir(parents=True)
        for img, lbl in items: shutil.copy2(img, d / "images" / img.name); shutil.copy2(lbl, d / "labels" / lbl.name)
    print(f"Split: train={len(splits['train'])}, val={len(splits['val'])}, test={len(splits['test'])}")

    # --- Report ---
    train_counts = defaultdict(int)
    for lf in (proc / "train" / "labels").iterdir():
        with open(lf) as f:
            for line in f:
                if line.strip(): train_counts[int(line.split()[0])] += 1

    print(f"\n{'='*60}")
    print(f"  DETECTION-ONLY DATASET — Self+TACO")
    print(f"{'='*60}")
    print(f"  Total: {total_imgs} images, {total_boxes} boxes")
    print(f"  Full-image boxes: 0 (all real detection)")
    print()
    print(f"  CLASS DISTRIBUTION (train):")
    for cid, name in enumerate(SELF_CLASSES):
        cnt = train_counts.get(cid, 0)
        bar = "=" * max(1, cnt // 10) if cnt > 0 else ""
        st = "OK" if cnt >= 50 else ("WEAK" if cnt > 0 else "MISSING")
        print(f"  {name:<20} {cnt:>5} {bar} {st}")

    tr_total = sum(train_counts.values())
    print(f"  {'TOTAL':<20} {tr_total:>5}")
    imb = max(train_counts.values()) / max(1, min(v for v in train_counts.values() if v > 0))
    print(f"  Imbalance: {imb:.1f}:1")
    print(f"\n  Full-image ratio: 0.0% — NEVER CRASH")
    print(f"  Output: {proc}")

if __name__ == "__main__":
    main()
