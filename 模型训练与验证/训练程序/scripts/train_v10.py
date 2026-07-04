def main():
    import sys, os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from train_logger import TrainingLogger
    from ultralytics import YOLO

    data_yaml = r"d:\AIview\模型训练与验证\训练程序\configs\dataset_v10.yaml"
    out_dir = r"d:\AIview\模型训练与验证\训练程序\runs"
    run_name = "v10_final"

    logger = TrainingLogger(run_name=run_name, output_dir=os.path.join(out_dir, run_name))

    print("=" * 60)
    print("  V10 — YOLOv11s, 8 classes, Mosaic=0.8, patience=15")
    print("  29,294 images / 35,883 boxes")
    print("  Full-img: 27.5% (below V1 safety 36%), box_margin=0.9")
    print("=" * 60)

    model = YOLO("yolo11s.pt")
    model.add_callback("on_fit_epoch_end", logger)

    results = model.train(
        data=data_yaml,
        epochs=300,
        imgsz=640,
        batch=16,
        device=0,
        workers=0,
        optimizer="AdamW",
        lr0=0.001,
        lrf=0.01,
        momentum=0.937,
        weight_decay=0.0005,
        warmup_epochs=10,
        cos_lr=True,
        amp=True,
        box=7.5,
        cls=0.5,
        dfl=1.5,
        mosaic=0.8,
        mixup=0.0,
        fliplr=0.5,
        scale=0.5,
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        erasing=0.2,
        close_mosaic=10,
        patience=15,
        save=True,
        save_period=5,
        project=out_dir,
        name=run_name,
        exist_ok=True,
        plots=True,
        verbose=True,
        seed=42,
    )

    best = logger.get_best()
    print(f"\nDone — best: {results.save_dir}/weights/best.pt")
    print(f"Best epoch: E{best['epoch']} mAP50={best['mAP50']:.4f}")

if __name__ == "__main__":
    from multiprocessing import freeze_support
    freeze_support()
    main()
