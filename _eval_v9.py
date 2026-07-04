def main():
    from ultralytics import YOLO

    data_yaml = r"d:\AIview\模型训练与验证\训练程序\configs\dataset_v9.yaml"
    model_path = r"d:\AIview\模型训练与验证\训练程序\runs\v9_safe\weights\best.pt"

    print("=" * 60)
    print("  Evaluating V9 best.pt (E52, mAP50=0.836)")
    print("=" * 60)

    model = YOLO(model_path)
    results = model.val(data=data_yaml, device=0, workers=0, split="test")

    box = results.box
    names = results.names

    f1 = 2 * box.mp * box.mr / (box.mp + box.mr + 1e-8)

    print(f"\n  OVERALL")
    print(f"  mAP50      = {box.map50:.4f}")
    print(f"  mAP50-95   = {box.map:.4f}")
    print(f"  Precision  = {box.mp:.4f}")
    print(f"  Recall     = {box.mr:.4f}")
    print(f"  F1         = {f1:.4f}")

    print(f"\n  PER-CLASS")
    print(f"  {'Class':<20} {'AP50':>8} {'P':>8} {'R':>8}")
    print(f"  {'-'*20} {'-'*8} {'-'*8} {'-'*8}")
    for cid in range(len(names)):
        name = names[cid]
        ap50 = box.ap50[cid] if len(box.ap50) > cid else 0
        p = box.p[cid] if len(box.p) > cid else 0
        r = box.r[cid] if len(box.r) > cid else 0
        print(f"  {name:<20} {ap50:>8.4f} {p:>8.4f} {r:>8.4f}")

    # Speed
    if hasattr(results, 'speed'):
        print(f"\n  Speed: {results.speed}")

if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()
    main()
