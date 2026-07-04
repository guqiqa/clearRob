def main():
    from ultralytics import YOLO
    data_yaml = r"d:\AIview\模型训练与验证\训练程序\configs\dataset_v10.yaml"
    model_path = r"d:\AIview\模型训练与验证\训练程序\runs\v10_final\weights\best.pt"
    print("=" * 60)
    print("  Evaluating V10 best.pt (E156)")
    print("=" * 60)
    model = YOLO(model_path)
    results = model.val(data=data_yaml, device=0, workers=0, split="test")
    box = results.box
    names = results.names
    mp_val = box.mp
    mr_val = box.mr
    f1 = 2 * mp_val * mr_val / (mp_val + mr_val + 1e-8)
    print(f"\n  OVERALL  mAP50={box.map50:.4f}  mAP50-95={box.map:.4f}  P={mp_val:.4f}  R={mr_val:.4f}  F1={f1:.4f}")
    header = "{:<20} {:>8} {:>8} {:>8}".format("Class", "AP50", "P", "R")
    print(f"\n  {header}")
    for cid in range(len(names)):
        ap50 = box.ap50[cid] if len(box.ap50) > cid else 0
        p = box.p[cid] if len(box.p) > cid else 0
        r = box.r[cid] if len(box.r) > cid else 0
        row = "{:<20} {:>8.4f} {:>8.4f} {:>8.4f}".format(names[cid], ap50, p, r)
        print(f"  {row}")
    # Speed
    s = results.speed
    print(f"\n  Speed: {s}")

if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()
    main()
