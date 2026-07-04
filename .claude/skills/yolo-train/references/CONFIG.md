# Training Configuration Reference

## Model Selection

| Model | Params | GFLOPS | Speed (RTX 5070 Ti) | Use Case |
|-------|--------|--------|---------------------|----------|
| YOLOv8n | 3.2M | 8.7G | ~2ms/img | Quick tests |
| YOLOv8s | 11.1M | 28.6G | ~3ms/img | Production baseline |
| YOLOv11n | 2.6M | 6.5G | ~1.5ms/img | Edge deployment |
| **YOLOv11s** | **9.4M** | **21.6G** | **~2.8ms/img** | **Recommended** |

YOLOv11s is ~25% faster than YOLOv8s with same accuracy. Use it unless you have a specific reason to stay on v8.

## Core Hyperparameters

### Learning Rate
| Param | Range | Default | Effect |
|-------|-------|---------|--------|
| lr0 | 0.0005-0.01 | **0.001** | Higher = faster convergence but riskier |
| lrf | 0.001-0.1 | **0.01** | Final LR = lr0 * lrf. Lower = smoother landing |
| warmup_epochs | 3-20 | **3 or 10** | 10 if Mosaic >= 0.8, else 3 |
| cos_lr | true/false | **true** | Cosine annealing from lr0 to lr0*lrf |

### Optimizer
| Param | Range | Default | Effect |
|-------|-------|---------|--------|
| optimizer | AdamW/SGD/Adam | **AdamW** | AdamW best for detection |
| momentum | 0.8-0.99 | **0.937** | Only for SGD |
| weight_decay | 0.0-0.001 | **0.0005** | L2 regularization |

### Data Augmentation
| Param | Range | Default | Effect |
|-------|-------|---------|--------|
| **mosaic** | 0.0-1.0 | **0.8** | 4-image拼接。安全前提：全图框<36% |
| mixup | 0.0-0.3 | **0.0** | 两图混合，不推荐与全图框数据共用 |
| fliplr | 0.0-0.5 | **0.5** | 水平翻转，所有对称类别安全 |
| scale | 0.1-0.9 | **0.5** | 随机缩放幅度 |
| hsv_h | 0.0-0.1 | **0.015** | 色调扰动 |
| hsv_s | 0.0-1.0 | **0.7** | 饱和度扰动 |
| hsv_v | 0.0-1.0 | **0.4** | 亮度扰动 |
| erasing | 0.0-0.5 | **0.2** | 随机遮挡比例 |
| close_mosaic | 0-20 | **10** | 最后N轮关闭Mosaic，0=不关 |

### Loss Weights
| Param | Range | Default | Effect |
|-------|-------|---------|--------|
| box | 5.0-12.0 | **7.5** | 定位loss权重。调高=更注重框精度 |
| cls | 0.3-1.0 | **0.5** | 分类loss权重。调高=更注重"是什么" |
| dfl | 1.0-2.0 | **1.5** | Distribution Focal Loss, 一般不调 |

### Training Control
| Param | Range | Default | Effect |
|-------|-------|---------|--------|
| epochs | 100-500 | **300** | 最大训练轮数 |
| patience | 10-100 | **15** | 连续N轮不破记录即停。15偏紧，50偏松 |
| batch | 8-32 | **16** | RTX 5070 Ti 12GB上限约24 |
| imgsz | 320-1280 | **640** | 越大越准但越慢越吃显存 |
| workers | 0-16 | **0 (Win)** / 8 (Linux) | Windows必须=0 |
| device | 0/cpu/0,1 | **0** | GPU编号 |
| amp | true/false | **true** | FP16混合精度，显存减半精度无损 |

### Regularization
| Param | Range | Default | Effect |
|-------|-------|---------|--------|
| dropout | 0.0-0.3 | **0.0** | 分类头dropout |
| label_smoothing | 0.0-0.1 | **0.0** | 标签平滑 |

## Recommended Configurations

### Conservative (Low Crash Risk)
```python
mosaic=0.0, mixup=0.0, patience=50, warmup_epochs=3
# Use when: full-image ratio 30-40%, unstable data
# Pro: almost never crashes
# Con: slow convergence, ~0.80 peak
```

### Balanced (V10 Proven)
```python
mosaic=0.8, mixup=0.0, patience=15, warmup_epochs=10,
close_mosaic=10, erasing=0.2
# Use when: full-image ratio < 30%, box_margin applied
# Pro: fast convergence, ~0.85 peak, no crash
# Con: requires clean data prep
```

### Aggressive (V1-style)
```python
mosaic=1.0, mixup=0.1, patience=50, warmup_epochs=3,
close_mosaic=10
# Use when: 100% real detection data, no full-image boxes at all
# Pro: highest potential peak (~0.90)
# Con: crashes if any full-image data present
```

## Per-Epoch Time Estimation

| Dataset Size | YOLOv11s | YOLOv8s |
|-------------|----------|----------|
| 20,000 imgs | ~6 min | ~10 min |
| 40,000 imgs | ~8 min | ~14 min |
| 60,000 imgs | ~12 min | ~20 min |
| 90,000 imgs | ~20 min | ~32 min |

RTX 5070 Ti Laptop, batch=16, imgsz=640, workers=0, Windows.
