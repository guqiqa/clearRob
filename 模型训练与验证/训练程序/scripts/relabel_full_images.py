"""
模型辅助标注 — 用已有模型对全图框数据重新标注，缩小框图。
单图处理（避免批量 crash 时一张坏图拖垮整批）。
"""

import argparse, os, sys, shutil
from pathlib import Path
from collections import defaultdict
from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

V1_TO_FINE = {
    0: [0, 1, 2, 3],  # recyclable → rec_plastic/rec_metal/rec_glass/rec_paper
    1: [4],            # kitchen_waste
    2: [5],            # hazardous
    3: [7],            # other_waste → waste_misc
    4: None,           # green_waste → DISCARD
    5: [8,9,10,11,12,13,14,15,16],  # large_obstacle → all road_*
    6: [17],           # pedestrian_pet → person
    7: [18],           # stain → stain_water
}

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def process_image(model, img_path, out_img_dir, out_lbl_dir, conf):
    """Single-image inference with error handling. Returns (ok, num_boxes)."""
    try:
        # Verify image integrity first
        img = Image.open(img_path)
        img.verify()
    except Exception:
        # Corrupt image — skip
        return False, 0

    try:
        results = model(str(img_path), device=0, conf=conf, max_det=50, verbose=False, stream=False)
    except Exception:
        return False, 0

    shutil.copy2(img_path, out_img_dir / img_path.name)

    if results is None or len(results) == 0:
        return True, 0

    r = results[0]
    boxes = r.boxes
    if boxes is None or len(boxes) == 0:
        # No detections → empty label
        with open(out_lbl_dir / f"{img_path.stem}.txt", "w") as f:
            pass
        return True, 0

    orig_w, orig_h = r.orig_shape[1], r.orig_shape[0]
    lines = []

    for box in boxes:
        cls_id = int(box.cls[0])
        box_conf = float(box.conf[0])
        if box_conf < conf:
            continue

        fine_ids = V1_TO_FINE.get(cls_id)
        if fine_ids is None:
            continue

        x1, y1, x2, y2 = box.xyxy[0].tolist()
        if len(fine_ids) > 1:
            fine_id = fine_ids[abs(hash(f"{img_path.name}_{x1:.0f}_{y1:.0f}")) % len(fine_ids)]
        else:
            fine_id = fine_ids[0]

        cx = max(0, min(1, (x1 + x2) / 2 / orig_w))
        cy = max(0, min(1, (y1 + y2) / 2 / orig_h))
        bw = max(0.001, min(1, (x2 - x1) / orig_w))
        bh = max(0.001, min(1, (y2 - y1) / orig_h))
        lines.append(f"{fine_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")

    with open(out_lbl_dir / f"{img_path.stem}.txt", "w") as f:
        f.writelines(lines)
    return True, len(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--sources", nargs="+", default=["garbages"])
    parser.add_argument("--conf", type=float, default=0.3)
    args = parser.parse_args()

    from ultralytics import YOLO
    print(f"Loading {args.model} ...")
    model = YOLO(args.model)

    output_base = PROJECT_ROOT.parent / "数据集" / "relabeled"
    if output_base.exists():
        shutil.rmtree(output_base)

    for src_name in args.sources:
        src_path = PROJECT_ROOT.parent / "数据集" / "raw" / src_name
        # Handle folder-based (garbages: Harmful/Kitchen/Other/Recyclable subdirs)
        # and csv-based (garbage_classify: train_data/*.txt + images)
        if (src_path / "Harmful").exists() and (src_path / "Kitchen").exists():
            # garbages folder format
            all_imgs = []
            for cls_dir in sorted(src_path.iterdir()):
                if not cls_dir.is_dir() or cls_dir.name.startswith("."): continue
                for f in cls_dir.iterdir():
                    if f.suffix.lower() in IMG_EXTS:
                        all_imgs.append(f)
            print(f"\n[{src_name}] folder format: {len(all_imgs)} images")
        elif (src_path / "train_data").exists():
            # garbage_classify csv format
            td = src_path / "train_data"
            all_imgs = [f for f in td.iterdir() if f.suffix.lower() in IMG_EXTS]
            print(f"\n[{src_name}] csv format: {len(all_imgs)} images")
        else:
            print(f"\n[{src_name}] unknown format, skipping")
            continue

        od = output_base / src_name
        oi = od / "images"; ol = od / "labels"
        oi.mkdir(parents=True, exist_ok=True); ol.mkdir(parents=True, exist_ok=True)

        ok, corr, boxes = 0, 0, 0
        for img_file in all_imgs:
            suc, nb = process_image(model, img_file, oi, ol, args.conf)
            if suc:
                ok += 1; boxes += nb
            else:
                corr += 1
            if (ok + corr) % 2000 == 0:
                print(f"  {ok+corr}/{len(all_imgs)}: {ok} ok, {corr} corrupt, {boxes} boxes")

        print(f"  Done: {ok} ok, {corr} corrupt, {boxes} detection boxes")

    print(f"\nOutput: {output_base}")
    print("Next: add relabeled/* to source_datasets_v7.yaml, disable garbages/garbage_classify")


if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()
    main()
