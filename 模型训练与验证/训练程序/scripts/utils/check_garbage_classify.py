import os
from collections import Counter

data_dir = r"D:\AIview\date\garbage_classify\train_data"
txts = [f for f in os.listdir(data_dir) if f.endswith(".txt")]

class_counter = Counter()
for tf in txts:
    with open(os.path.join(data_dir, tf), "r", encoding="utf-8") as f:
        line = f.read().strip()
    if "," in line:
        cls_id = int(line.split(",")[1].strip())
    else:
        cls_id = int(line.split()[0])
    class_counter[cls_id] += 1

print("Classification categories:")
for cid in sorted(class_counter):
    count = class_counter[cid]
    pct = count / sum(class_counter.values()) * 100
    print(f"  class {cid:>2}: {count:>5} images ({pct:.1f}%)")

print(f"\nTotal images: {sum(class_counter.values())}")
print(f"Unique classes: {len(class_counter)}")
print(f"Min per class: {min(class_counter.values())}")
print(f"Max per class: {max(class_counter.values())}")
print(f"Format: Image Classification")
print(f"Label: filename.jpg, class_id")
print(f"No bounding boxes — classification, NOT detection")

# Image check
from PIL import Image
import random
jpgs = [f for f in os.listdir(data_dir) if f.endswith(".jpg")]
bad = 0
for img_file in random.sample(jpgs, 50):
    path = os.path.join(data_dir, img_file)
    try:
        img = Image.open(path)
        img.verify()
    except:
        bad += 1
        print(f"  CORRUPT: {img_file}")
print(f"Image check: {50-bad}/50 OK, {bad} corrupt")

# Image size consistency
sizes = Counter()
for img_file in random.sample(jpgs, 30):
    path = os.path.join(data_dir, img_file)
    img = Image.open(path)
    sizes[img.size] += 1
print(f"\nImage sizes:")
for size, cnt in sizes.most_common():
    print(f"  {size[0]}x{size[1]}: {cnt}")
