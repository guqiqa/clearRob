# Cleaning Vision V13 — 智能环卫机器人视觉检测

YOLO11s ONNX 推理引擎，8 类垃圾/障碍物检测。
**零 ROS2 依赖**，可在任意 Ubuntu/Debian Linux 系统上独立运行。

```
模型:     YOLO11s (9.4M params, 21.3 GFLOPs)
类别:     可回收 / 厨余 / 有害 / 其他垃圾 / 绿化垃圾 / 行人&宠物 / 障碍物&车辆 / 污渍&积水
精度:     mAP50 = 0.783,  Recall = 0.725
输入:     RGB 图像, 640×640 (自动 letterbox)
输出:     检测框 + 类别 + 置信度
```

---

## 快速开始

### 1. 安装依赖

```bash
cd cleaning_vision_v13

# 基础安装 (CPU 推理)
pip install -r requirements.txt

# 如需 GPU 推理 (CUDA 11.8+)
pip install onnxruntime-gpu==1.15.1
```

### 2. 测试一张图片

```bash
python detect.py --image test.jpg --display
```

### 3. 常见用法

```bash
# 单张图片 → JSON 输出
python detect.py -i photo.jpg --json

# 图片 → 保存标注结果
python detect.py -i photo.jpg --save --json

# 批量处理文件夹
python detect.py --dir ./images/ --save

# USB 摄像头实时检测
python detect.py --camera 0

# 视频文件 → 导出标注视频
python detect.py --video road.mp4 --save

# 调整检测灵敏度
python detect.py -i photo.jpg --conf 0.5 --iou 0.3
```

---

## 文件结构

```
cleaning_vision_v13/
├── model/
│   ├── best.onnx          # YOLO11s ONNX 模型 (37 MB)
│   └── config.yaml        # 模型元信息 (类别、输入规格、性能指标)
├── yolo_detector.py       # ONNX 推理引擎 (可独立复用)
├── detect.py              # 命令行入口 (图片/文件夹/摄像头/视频)
├── requirements.txt       # Python 依赖
└── README.md
```

---

## 检测类别

| ID | 类别 | 中文 | 策略 |
|----|------|------|------|
| 0 | `recyclable` | 可回收垃圾 | 减速 → 扫入收集仓 |
| 1 | `kitchen_waste` | 厨余垃圾 | 先吸 → 后拖 |
| 2 | `hazardous` | 有害垃圾 | 单独收集 + 告警 |
| 3 | `other_waste` | 其他垃圾 | 直接吸扫 |
| 4 | `green_waste` | 绿化垃圾 | 直接吸扫 |
| 5 | `pedestrian` | 行人/宠物 | 减速/绕行/等待 |
| 6 | `obstacle` | 障碍物/车辆 | 绕行避让，不清扫 |
| 7 | `stain` | 污渍/积水 | 洒水重拖 / 绕行 |

---

## Python API (编程调用)

```python
import cv2
from yolo_detector import YOLODetector

# 加载模型
detector = YOLODetector(
    model_path="model/best.onnx",
    class_names=["recyclable", "kitchen_waste", "hazardous", "other_waste",
                 "green_waste", "pedestrian", "obstacle", "stain"],
    conf_threshold=0.25,
    iou_threshold=0.45,
)
detector.load()

# 推理
img = cv2.imread("photo.jpg")
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
detections, inference_ms = detector.detect(img_rgb)

for d in detections:
    print(f"{d.class_name}: conf={d.confidence:.2f}, "
          f"box=({d.xmin},{d.ymin},{d.xmax},{d.ymax})")

detector.unload()
```

---

## 系统要求

| 项目 | 最低 | 推荐 |
|------|------|------|
| 操作系统 | Ubuntu 20.04 | Ubuntu 22.04 |
| Python | 3.8 | 3.10+ |
| 内存 | 2 GB | 4 GB |
| 磁盘 | 100 MB | 200 MB |
| GPU (可选) | — | NVIDIA + CUDA 11.8+ |

**CPU 推理性能：** ~150-300 ms/帧 (取决于 CPU)
**GPU 推理性能：** ~10-20 ms/帧 (NVIDIA 显卡)

---

## 训练来源

- 训练代码: `runs/v13_8cls`
- 基础权重: `yolo11s.pt`
- 训练轮数: 186 epochs (early stop, patience=30)
- 数据集: TACO + UAVVaste + TrashNet + trainval_voc + street_obstacle + car_identify + pedestrian_coco + puddle + self_collected
- 导出格式: ONNX opset 17, simplified with onnxslim
