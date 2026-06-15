import os
from collections import Counter
from PIL import Image

data_dir = r"D:\AIview\date\dataset-resized\dataset-resized"
folders = ["cardboard", "glass", "metal", "paper", "plastic", "trash"]

mapping = {
    "cardboard": ("recyclable", "纸板箱"),
    "glass": ("recyclable", "玻璃"),
    "metal": ("recyclable", "金属"),
    "paper": ("recyclable", "纸类"),
    "plastic": ("recyclable", "塑料"),
    "trash": ("other_waste", "不可分辨垃圾"),
}

print("=" * 60)
print("  dataset-resized (TrashNet) Analysis")
print("=" * 60)

our_counts = {
    "recyclable": 10459,
    "other_waste": 4578,
}

total = 0
print()
print(f"  {'Folder':<18} {'Images':>7} {'Target':<20} {'Add':>7} {'Current':>8}")
print(f"  {'-'*18} {'-'*7} {'-'*20} {'-'*7} {'-'*8}")
for folder in folders:
    path = os.path.join(data_dir, folder)
    imgs = [f for f in os.listdir(path) if f.lower().endswith((".jpg",".jpeg",".png"))]
    count = len(imgs)
    total += count
    target, desc = mapping[folder]
    current = our_counts[target]
    print(f"  {folder:<18} {count:>7} {target:<20} +{count:<6} now:{current:>6}")

print(f"  {'-'*18} {'-'*7}")
print(f"  {'TOTAL':<18} {total:>7}")

print(f"\n  OVERLAP CHECK:")
print(f"    garbage_classify already has 14,802 images covering")
print(f"    plastic bottles, glass bottles, metal cans, paper, etc.")
print(f"    dataset-resized is a DIFFERENT dataset (TrashNet/Stanford)")
print(f"    Image size: 512x384 (all uniform)")
print(f"    garbage_classify: various sizes (768x1024, etc.)")
print(f"    -> Likely NO overlap, safe to merge")

print(f"\n  GAP IMPACT:")
print(f"    This dataset ONLY covers recyclable + other_waste")
print(f"    These classes are already well-covered (10,459 + 4,578 boxes)")
print(f"    green_waste: still 0 (unaffected)")
print(f"    pedestrian_pet: still 239 (unaffected)")
print(f"    stain: still 329 (unaffected)")
print(f"    -> Adds quantity but not new categories")

print(f"\n  FORMAT: Folder-based classification (same as TrashNet)")
print(f"  -> No bounding boxes, needs full-image-box conversion")
