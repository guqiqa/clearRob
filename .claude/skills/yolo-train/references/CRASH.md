# Crash Diagnosis Reference

## Table of Contents
1. Crash pattern identification
2. Root cause checklist
3. Fix recipes
4. Full-image-box threshold theory

---

## 1. Crash Pattern Identification

### Type A: Slow Decline (overfitting)

```
Epoch  mAP50  val_cls
E50    0.835  0.502  ← peak
E60    0.831  0.520  ← val_cls rising slowly
E70    0.801  0.639
E80    0.462  3.525  ← spike
E85    0.006  ∞      ← dead
```

Symptom: val_cls rises 0.01-0.03 per epoch for 20+ epochs, then accelerates to spike.
Root cause: full-image-box ratio > 36%.
Fix: reduce full-image sources or add more real detection data.

### Type B: Sudden Collapse (gradient explosion)

```
Epoch  mAP50  val_cls
E20    0.808  0.435  ← peak
E21    0.751  0.537
E22    0.000  ∞      ← instant death within 8 epochs
```

Symptom: mAP drops to 0 within 10 epochs of first val_cls deviation.
Root cause: full-image-box ratio > 71%.
Fix: drastic reduction of full-image sources (halve or more).

### Type C: No Crash (healthy)

```
Epoch  mAP50  val_cls
E50    0.820  0.590
E100   0.832  0.533
E150   0.836  0.528  ← stable plateau
E170   0.836  0.545  ← early stop
```

Symptom: val_cls stays below 0.60, mAP plateaus naturally.
Diagnosis: data is clean, training is healthy.

---

## 2. Root Cause Checklist

Run this checklist when a crash is detected:

### Dataset
- [ ] Full-image-box ratio: count boxes with area > 99%. If > 36%, this is the cause.
- [ ] Class imbalance: if max/min > 20:1, underrepresented classes may trigger collapse
- [ ] Empty labels: images with no annotations at all confuse the model
- [ ] Corrupt images: check with PIL Image.verify()

### Training Config
- [ ] Mosaic value: > 0 + high full-image ratio = accelerated crash
- [ ] val_cls_loss: single-epoch jump > 3.0 = gradient explosion
- [ ] train_cls vs val_cls gap: val exceeding train = overfitting begins
- [ ] Learning rate: too high (diverging) or too low (stuck)

### Code
- [ ] Dataset YAML path is absolute (not relative)
- [ ] nc matches actual number of classes in labels
- [ ] class IDs in labels are 0-indexed and contiguous
- [ ] workers=0 on Windows

---

## 3. Fix Recipes

### Fix 1: Reduce Full-Image Boxes

```yaml
# In source_datasets.yaml, reduce classification sources:
- name: garbages
  samples_per_class: 2800  # was 5000+

- name: leaf_classify
  samples_per_class: 2800  # was 5000+
```

### Fix 2: Add Box Margin

```yaml
- name: garbages
  box_margin: 0.9  # 5% border on each side, breaks "whole image = object"
```

### Fix 3: Cap Overrepresented Classes

```yaml
- name: datas
  per_class_cap:
    pedestrian_pet: 3   # max 3 boxes per image
    road_obstacle: 5    # max 5 boxes per image
```

### Fix 4: Disable Mosaic Temporarily

If crash happens in first 30 epochs with Mosaic > 0:
- Set Mosaic = 0.0
- Train 10 epochs
- If no crash: Mosaic was accelerating gradient conflict
- If still crashes: data is fundamentally broken

### Fix 5: Warmup Extension

If crash happens during or right after warmup:
- Increase warmup_epochs from 3 to 10
- This gives the optimizer more time to stabilize before full learning rate

---

## 4. Full-Image-Box Threshold Theory

From 10 rounds of empirical testing:

| Full-Image % | Survives? | Peak mAP50 | Notes |
|-------------|-----------|-----------|-------|
| 0% | Yes | ~0.70 | Only real detection data |
| 27.5% | **Yes** | **0.848** | V10: complete convergence |
| 28-36% | Likely | 0.78-0.84 | V5/V6 range, not yet fully tested |
| 36-45% | **No** | 0.83-0.89 | Crashes E50-E75 |
| 45-63% | No | 0.65-0.83 | Crashes E60-E100 |
| > 63% | No | 0.65-0.81 | Crashes E28-E152 |

**Safe zone: < 36%.** The V1 model survived E52 at 36% but crashed at E63.
Below 30% (V10 at 27.5%) completes full training with no crash.

The mechanism: full-image boxes provide contradictory gradient signals.
A `0.5 0.5 1.0 1.0` box tells the model "the entire 640x640 image is one object."
When Mosaic combines 4 such images, 4 contradictory "full-image" labels overlap
in a single training sample, creating gradient explosion that accumulates over epochs.
