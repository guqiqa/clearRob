"""Evaluate V10, V11, V12 best models on their respective test sets."""
from ultralytics import YOLO

versions = [
    ("V10", r"d:\AIview\模型训练与验证\训练程序\runs\v10_final\weights\best.pt",
           r"d:\AIview\模型训练与验证\训练程序\configs\dataset_v10.yaml"),
    ("V11", r"d:\AIview\模型训练与验证\训练程序\runs\v11_final\weights\best.pt",
           r"d:\AIview\模型训练与验证\训练程序\configs\dataset_v11.yaml"),
    ("V12", r"d:\AIview\模型训练与验证\训练程序\runs\v12_full\weights\best.pt",
           r"d:\AIview\模型训练与验证\训练程序\configs\dataset_v12.yaml"),
    ("V9",  r"d:\AIview\模型训练与验证\训练程序\runs\v9_safe\weights\best.pt",
           r"d:\AIview\模型训练与验证\训练程序\configs\dataset_v9.yaml"),
]

results_all = {}
for label, model_path, data_yaml in versions:
    print(f"\n{'='*60}")
    print(f"  Evaluating {label} — {model_path}")
    print(f"  Data: {data_yaml}")
    print(f"{'='*60}")
    model = YOLO(model_path)
    results = model.val(data=data_yaml, device=0, workers=0, split="test", verbose=False)
    box = results.box
    names = results.names

    mp_val = box.mp; mr_val = box.mr
    f1 = 2 * mp_val * mr_val / (mp_val + mr_val + 1e-8)
    print(f"\n  OVERALL  mAP50={box.map50:.4f}  mAP50-95={box.map:.4f}  P={mp_val:.4f}  R={mr_val:.4f}  F1={f1:.4f}")
    header = f"  {'Class':<20} {'AP50':>8} {'P':>8} {'R':>8}"
    print(f"\n  {header}")
    print(f"  {'-'*20} {'-'*8} {'-'*8} {'-'*8}")

    per_class = {}
    for cid in range(len(names)):
        ap50 = float(box.ap50[cid]) if len(box.ap50) > cid else 0
        p = float(box.p[cid]) if len(box.p) > cid else 0
        r = float(box.r[cid]) if len(box.r) > cid else 0
        print(f"  {names[cid]:<20} {ap50:>8.4f} {p:>8.4f} {r:>8.4f}")
        per_class[names[cid]] = ap50
    results_all[label] = {"mAP50": float(box.map50), "mAP50-95": float(box.map),
                          "P": float(mp_val), "R": float(mr_val), "F1": float(f1),
                          "per_class": per_class}

# Summary comparison
print(f"\n{'='*60}")
print(f"  CROSS-VERSION COMPARISON (test set)")
print(f"{'='*60}")
print(f"  {'Class':<20} {'V10':>8} {'V11':>8} {'V12':>8} {'Best':>10}")
print(f"  {'-'*20} {'-'*8} {'-'*8} {'-'*8} {'-'*10}")
names_list = ['recyclable','kitchen_waste','hazardous','other_waste','green_waste','road_obstacle','pedestrian_pet','stain']
for cls in names_list:
    v10 = results_all['V10']['per_class'].get(cls, 0)
    v11 = results_all['V11']['per_class'].get(cls, 0)
    v12 = results_all['V12']['per_class'].get(cls, 0)
    best = max(v10, v11, v12)
    best_label = ['V10','V11','V12','V9'][[v10,v11,v12,v9].index(best)] if best > 0 else '-'
    print(f"  {cls:<20} {v10:>8.4f} {v11:>8.4f} {v12:>8.4f} {best_label:>10}")

print(f"  {'-'*20} {'-'*8} {'-'*8} {'-'*8} {'-'*10}")
for metric in ['mAP50','mAP50-95','P','R','F1']:
    v10 = results_all['V10'][metric]
    v11 = results_all['V11'][metric]
    v12 = results_all['V12'][metric]
    best = max(v10, v11, v12)
    best_label = ['V10','V11','V12'][[v10,v11,v12].index(best)]
    print(f"  {metric:<20} {v10:>8.4f} {v11:>8.4f} {v12:>8.4f} {best_label:>10}")

print()
