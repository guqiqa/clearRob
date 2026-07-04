---
name: yolo-train
description: >
  Full-cycle YOLO training automation — dataset preparation, training configuration,
  progress monitoring, evaluation, and record keeping. Covers the complete workflow
  from raw data to deployed model. Use when the user needs to: (1) prepare a YOLO
  dataset from multiple sources with label mapping, (2) configure and launch training
  with informed parameter choices, (3) monitor training progress and diagnose issues,
  (4) evaluate a trained model with per-class metrics, (5) update training records,
  or (6) diagnose and fix training crashes. Supports both Ultralytics YOLOv8/YOLOv11.
---

# YOLO Training Automation

Complete YOLO training workflow with crash prevention built in from hard-earned experience.

## Quick Decision Checklist

Before starting, confirm these with the user:

1. **Model**: YOLOv8s or YOLOv11s? (v11s faster, same accuracy from our testing)
2. **Classes**: How many? What are they? → write `dataset.yaml`
3. **Mosaic**: 0.8 recommended if data is clean. 0.0 if dataset has full-image-box issues (see [CRASH.md](references/CRASH.md))
4. **Patience**: 15-20 for production, 50+ for exploration
5. **Epochs**: 200-300
6. **Warmup**: 3 (standard) or 10 (if Mosaic >= 0.8)
7. **Full-image-box ratio**: Must stay below 36% of total boxes. See [CRASH.md](references/CRASH.md) for why.

## Workflow

### Phase 1: Dataset Preparation

```
Raw sources → label_mapping.yaml → prepare_dataset.py → processed/
```

1. Survey all data sources in the user's data directory
2. For each source, determine format (COCO, VOC, YOLO txt, folder classification, CSV)
3. Create `label_mapping.yaml` — one mapping per source, all converging to target classes
4. Create `source_datasets.yaml` — enable/disable sources, set per-class caps, set `samples_per_class` for balance
5. Run preparation — count classes, verify full-image-box ratio < 36%, report distribution

**Key constraint**: Full-image-box ratio = (boxes with area > 99%) / total boxes. Must be < 36%.
If over, reduce `samples_per_class` on classification sources until under threshold.

**Script pattern** — use this template for the preparation script:
- Read label_mapping.yaml for target classes and source-specific mappings
- Handler functions for each source format (COCO, YOLO, VOC, folder, CSV, LabelMe)
- `per_class_cap` support for controlling over-represented classes
- `samples_per_class` for classification sources (random sampling)
- `box_margin` support (e.g., 0.9 = 5% border on each side) for classification→detection conversion
- Final split: 70/15/15 train/val/test
- Print class distribution, full-image ratio, and data source composition

### Phase 2: Training Configuration

1. Create `dataset.yaml` with absolute path, class names, nc
2. Write training script with callbacks. Template:

```python
from train_logger import TrainingLogger
from ultralytics import YOLO

logger = TrainingLogger(run_name="vX_name", output_dir=out_dir)
model = YOLO("yolo11s.pt")  # or yolov8s.pt
model.add_callback("on_fit_epoch_end", logger)

results = model.train(
    data=data_yaml, epochs=300, imgsz=640, batch=16,
    device=0, workers=0,  # workers=0 for Windows
    optimizer="AdamW", lr0=0.001, lrf=0.01,
    momentum=0.937, weight_decay=0.0005,
    warmup_epochs=10, cos_lr=True, amp=True,
    box=7.5, cls=0.5, dfl=1.5,
    mosaic=0.8, mixup=0.0,
    fliplr=0.5, scale=0.5, hsv_h=0.015, hsv_s=0.7, hsv_v=0.4,
    erasing=0.2, close_mosaic=10,
    patience=15, save=True, save_period=5,
    project=out_dir, name=run_name, exist_ok=True,
    plots=True, verbose=True, seed=42,
)
```

3. The `TrainingLogger` callback (see `scripts/train_logger.py`) writes per-epoch JSONL and a summary file.

### Phase 3: Monitoring

Check training status with:

```python
import csv
with open("runs/{name}/results.csv") as f:
    rows = list(csv.DictReader(f))
n = len(rows)
m50 = [float(r['metrics/mAP50(B)']) for r in rows]
peak = max(m50)
peak_ep = max(range(len(m50)), key=lambda i: m50[i]) + 1
```

**Health checks** (every check):
- val_cls_loss < 2.0 and not rising sharply
- mAP50 not declining more than 0.02 from peak
- No single-epoch drop > 0.05 in mAP50

**Crash warning signs** (see [CRASH.md](references/CRASH.md)):
- val_cls crosses above train_cls and diverges
- val_cls spikes > 3.0 in a single epoch
- mAP drops to 0 within 10 epochs of val_cls spike

### Phase 4: Evaluation

```python
from ultralytics import YOLO
model = YOLO("runs/{name}/weights/best.pt")
results = model.val(data=data_yaml, device=0, workers=0, split="test")
box = results.box
# Print overall + per-class AP50/P/R
```

### Phase 5: Record Keeping

Update the training record with:
- Configuration table row (model, classes, data size, Mosaic, patience, etc.)
- Results table row (peak mAP50, epoch, mAP50-95, R, P, crash status, time)
- Per-class AP50 table
- Epoch log (key milestones: E1, E5, E10, E15, E20, E30, E50, E100, peak, final)
- If crashed: crash timeline with val_cls inflection point
- Ranking vs previous rounds

## Key Design Rules (from 10 rounds of trial and error)

1. **Mosaic=0 is safe but slow.** Mosaic=0.8 is fast, but only safe when full-image ratio < 36%.
2. **Full-image boxes (0.5, 0.5, 1.0, 1.0) must stay below 36% of total boxes.** Above this, training will eventually crash. Every round above 36% crashed. Every round below did not.
3. **Use box_margin=0.9 for classification-converted data.** Instead of a full-image box, use `0.5 0.5 0.9 0.9` to leave a 5% border. This breaks the "entire image = one object" illusion.
4. **Windows workers=0 is required.** Multi-process data loading crashes on Windows.
5. **Always add `freeze_support()`** in `if __name__ == "__main__"` for Windows.
6. **Use `model.add_callback()` not `callbacks=`** — the `callbacks` kwarg is not supported by ultralytics.
7. **patience=15 with save_period=5** ensures you don't waste GPU hours after convergence.
8. **warmup=10 for Mosaic >= 0.8.** Gives the model time to adapt to aggressive augmentation.

## Reference Files

- **[CRASH.md](references/CRASH.md)** — Complete crash diagnosis guide: symptoms, root causes, fix recipes
- **[CONFIG.md](references/CONFIG.md)** — Full parameter reference with trade-offs and recommended ranges
