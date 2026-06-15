def main():
    from ultralytics import YOLO

    data_yaml = r"d:\AIview\模型训练与验证\训练程序\configs\dataset.yaml"
    out_dir = r"d:\AIview\模型训练与验证\训练程序\runs"

    print("=" * 60)
    print("  v5 — YOLOv8s from scratch, NO mosaic, balanced data")
    print("  32,226 train / 6,906 val / 60,694 boxes / 8 classes")
    print("  mosaic=0.0 (fixes full-image box collision)")
    print("=" * 60)

    model = YOLO("yolov8s.pt")

    results = model.train(
        data=data_yaml,
        epochs=200,
        imgsz=640,
        batch=16,
        device=0,
        workers=0,
        optimizer="AdamW",
        lr0=0.001,
        lrf=0.01,
        momentum=0.937,
        weight_decay=0.0005,
        warmup_epochs=3,
        cos_lr=True,
        amp=True,
        box=7.5,
        cls=0.5,
        dfl=1.5,
        mosaic=0.0,               # OFF — eliminates full-image box collision
        mixup=0.0,                # OFF — conflicts with detection training
        fliplr=0.5,
        scale=0.5,
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        erasing=0.2,
        close_mosaic=0,            # not needed with mosaic=0
        patience=50,
        save=True,
        save_period=10,
        project=out_dir,
        name="balanced_v5",
        exist_ok=True,
        plots=True,
        verbose=True,
        seed=42,
    )

    print(f"\nDone — best: {results.save_dir}/weights/best.pt")

if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()
    main()
