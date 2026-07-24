#!/usr/bin/env python3
"""Download ros-humble-* ARM64 .deb from Tsinghua ROS2 mirror (fast in China)."""
import urllib.request, gzip, re, os, sys

BASE_INDEX = 'https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/dists/jammy/main/binary-arm64/Packages.gz'
BASE_DEB = 'https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu'
OUT = sys.argv[1] if len(sys.argv) > 1 else "phase1_deps/debs"
os.makedirs(OUT, exist_ok=True)

print("Fetching package index from Tsinghua mirror...")
req = urllib.request.Request(BASE_INDEX, headers={'User-Agent': 'phase1/1.0'})
with urllib.request.urlopen(req, timeout=120) as r:
    data = gzip.decompress(r.read()).decode('utf-8', errors='replace')

# Parse all ros-humble-* entries
entries = data.strip().split('\n\n')
targets = []
for entry in entries:
    pkg_m = re.search(r'^Package:\s*(\S+)', entry, re.MULTILINE)
    fn_m = re.search(r'^Filename:\s*(\S+)', entry, re.MULTILINE)
    sz_m = re.search(r'^Size:\s*(\d+)', entry, re.MULTILINE)
    if pkg_m and fn_m and sz_m:
        name = pkg_m.group(1)
        if 'humble' in name.lower():
            targets.append((name, f'{BASE_DEB}/{fn_m.group(1)}', int(sz_m.group(1))))

total_mb = sum(s[2] for s in targets) / 1024 / 1024
print(f"Found {len(targets)} ros-humble-* packages ({total_mb:.0f} MB)")

# Download
for i, (name, url, size) in enumerate(targets):
    fname = os.path.join(OUT, os.path.basename(url))
    if os.path.exists(fname) and os.path.getsize(fname) > 0:
        continue
    try:
        req = urllib.request.Request(url, headers={'User-Agent': 'phase1/1.0'})
        with urllib.request.urlopen(req, timeout=120) as r:
            blob = r.read()
        with open(fname, 'wb') as f:
            f.write(blob)
        if (i + 1) % 50 == 0:
            done_mb = sum(os.path.getsize(os.path.join(OUT, f))
                         for f in os.listdir(OUT)) / 1024 / 1024
            print(f"  [{i+1}/{len(targets)}] {done_mb:.0f} MB")
    except Exception as e:
        print(f"  FAIL: {name} — {e}")

done_mb = sum(os.path.getsize(os.path.join(OUT, f)) for f in os.listdir(OUT)) / 1024 / 1024
n = len(os.listdir(OUT))
print(f"\nDone: {n} files, {done_mb:.0f} MB in {OUT}/")
