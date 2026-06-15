import sys
sys.path.insert(0, r"d:\AIview\模型训练与验证\训练程序")

print("=" * 70)
print("  Old 22-class street obstacle -> Target 8-class garbage mapping")
print("=" * 70)

# Our 8 target classes
TARGETS = [
    "recyclable", "kitchen_waste", "hazardous", "other_waste",
    "green_waste", "large_obstacle", "pedestrian_pet", "stain"
]

# Analysis: (cid, name, count, mapped_target, risk_flag, reason)
analysis = [
    (0, "chair", 126, "discard", "OK", "indoor furniture, not garbage"),
    (1, "fence", 73, "discard", "OK", "fixed infrastructure"),
    (2, "garbage bin", 56, "large_obstacle", "RISK",
     "BIN = container, NOT garbage. Model may learn to detect bins instead of trash inside"),
    (3, "pole", 208, "discard", "OK", "utility poles, fixed"),
    (4, "obstacle", 400, "large_obstacle", "GOOD",
     "FILLS large_obstacle gap! Labelled as obstacle in source"),
    (5, "tree", 207, "green_waste", "RISK",
     "Living trees != green_waste (fallen leaves/branches). Will confuse model"),
    (6, "plant pot", 159, "discard", "RISK",
     "A pot is a container, not garbage itself"),
    (7, "stair", 95, "discard", "OK", "building structure"),
    (8, "bad road", 212, "discard", "OK", "road condition, not object"),
    (9, "street vendor", 111, "pedestrian_pet", "GOOD",
     "FILLS pedestrian_pet gap! Vendor = person"),
    (10, "drain", 100, "discard", "OK", "fixed drainage"),
    (11, "pothole", 230, "discard", "OK", "road surface defect"),
    (12, "gate barrier", 48, "discard", "OK", "fixed gate"),
    (13, "puddle", 237, "stain", "GOOD",
     "FILLS stain gap! Puddle = water stain on ground"),
    (14, "motorcycle", 251, "discard", "OK", "vehicle, not sanitation target"),
    (15, "car", 364, "discard", "OK", "vehicle, not sanitation target"),
    (16, "left turn", 70, "discard", "OK", "traffic sign"),
    (17, "pedestrian", 54, "pedestrian_pet", "GOOD",
     "FILLS pedestrian_pet gap! This is a person"),
    (18, "roadblock", 177, "large_obstacle", "GOOD",
     "FILLS large_obstacle gap! Roadblock = obstacle"),
    (19, "door", 37, "discard", "OK", "not relevant"),
    (20, "right turn", 66, "discard", "OK", "traffic sign"),
    (21, "zebra cross", 394, "discard", "OK", "crosswalk, not object"),
]

print()
print("LEGEND: GOOD=fill gaps safely | RISK=may harm model | OK=discard (noise)")
print()

good = []
risk = []
discard = []

for cid, name, count, target, flag, reason in analysis:
    tag = "GOOD" if flag == "GOOD" else ("RISK" if flag == "RISK" else "---")
    print(f"  {tag:4} | {name:<18} -> {target:<20} | {count:>4}  | {reason}")

    if flag == "GOOD":
        good.append((cid, name, count, target))
    elif flag == "RISK":
        risk.append((cid, name, count, target, reason))
    else:
        discard.append((cid, name, count))

print()
print("=" * 70)
print("  SUMMARY")
print("=" * 70)

good_boxes = sum(c for _, _, c, _ in good)
risk_boxes = sum(c for _, _, c, _, _ in risk)
discard_boxes = sum(c for _, _, c in discard)

print()
print(f"  SAFE TO MERGE: {len(good)} classes, {good_boxes} boxes")
print(f"  RISKY (needs decision): {len(risk)} classes, {risk_boxes} boxes")
print(f"  DISCARD: {len(discard)} classes, {discard_boxes} boxes")
print(f"  TOTAL: 3675 boxes")

print()
print("  --- SAFE (gap-filling) ---")
gap_before = {"large_obstacle": 0, "pedestrian_pet": 0, "stain": 0}
for cid, name, count, target in good:
    gap_before[target] += count
    print(f"    {name} -> {target} (+{count} boxes)")

print()
print("  --- RISKY (your decision) ---")
for cid, name, count, target, reason in risk:
    print(f"    {name} -> {target} (+{count} boxes)")
    print(f"      REASON: {reason}")
    print()

print("  --- Current gaps filled by this merge ---")
print(f"    large_obstacle: 0 -> {gap_before['large_obstacle']} boxes")
print(f"    pedestrian_pet: 0 -> {gap_before['pedestrian_pet']} boxes")
print(f"    stain:          0 -> {gap_before['stain']} boxes")

print()
print("=" * 70)
print("  RECOMMENDATION: Merge only GOOD classes (5), discard RISK + discard")
print("  RISK classes need your review before merging")
print("=" * 70)
