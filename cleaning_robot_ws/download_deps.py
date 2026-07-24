#!/usr/bin/env python3
"""
ARM64 ROS2 Humble offline package downloader.

Downloads all .deb packages needed for ros-humble-ros-base
and Python dependencies (python-can, pyserial, numpy, pyyaml)
for aarch64 Ubuntu 22.04 (Jammy).

Usage: python download_deps.py
Output: ./phase1_deps/ directory with all .deb files + pip wheels
"""

import gzip
import os
import re
import ssl
import subprocess
import sys
import urllib.request
from collections import deque

# SSL context that skips verification for repos with cert issues
_SSL_CTX = ssl._create_unverified_context()

# ---- Config ----
OUTPUT_DIR = "phase1_deps"
DEB_DIR = os.path.join(OUTPUT_DIR, "debs")
WHEEL_DIR = os.path.join(OUTPUT_DIR, "wheels")

# Repos to scan for packages
REPOS = [
    {
        "name": "ros2",
        "packages_url": "https://packages.ros.org/ros2/ubuntu/dists/jammy/main/binary-arm64/Packages.gz",
        "base_url": "https://packages.ros.org/ros2/ubuntu",
    },
    {
        "name": "ubuntu-main",
        "packages_url": "http://ports.ubuntu.com/dists/jammy/main/binary-arm64/Packages.gz",
        "base_url": "http://ports.ubuntu.com",
    },
    {
        "name": "ubuntu-universe",
        "packages_url": "http://ports.ubuntu.com/dists/jammy/universe/binary-arm64/Packages.gz",
        "base_url": "http://ports.ubuntu.com",
    },
]

# Packages we want to install
TARGET_PACKAGES = [
    "ros-humble-ros-base",
]

# Additional system packages needed
EXTRA_PACKAGES = [
    "python3-pip",
    "python3-numpy",
    "python3-yaml",
]

PIP_PACKAGES = [
    "python-can",
    "pyserial",
    "pyyaml",
    "numpy",
]

# Base system packages that are ALREADY on the device (don't download)
# These come from the base Ubuntu 22.04 ARM64 image
SKIP_IF_MISSING = True  # Download everything needed


def fetch_packages_gz(url: str) -> str:
    """Download and decompress a Packages.gz file, return text content."""
    print(f"  Fetching: {url}")
    req = urllib.request.Request(url, headers={"User-Agent": "phase1-downloader/1.0"})
    with urllib.request.urlopen(req, timeout=60, context=_SSL_CTX) as resp:
        data = resp.read()
    return gzip.decompress(data).decode("utf-8", errors="replace")


def parse_packages(text: str) -> dict:
    """Parse Debian Packages file into dict of {package_name: {field: value}}."""
    packages = {}
    current = {}
    current_name = None

    for line in text.split("\n"):
        if line.strip() == "":
            if current_name:
                packages[current_name] = current
            current = {}
            current_name = None
            continue

        if line.startswith(" "):
            # Continuation line
            if current_name:
                for key in current:
                    current[key] += " " + line.strip()
                    break
            continue

        if ":" in line:
            key, _, value = line.partition(":")
            key = key.strip()
            value = value.strip()
            current[key] = value
            if key == "Package":
                current_name = value

    if current_name:
        packages[current_name] = current

    return packages


def parse_depends(dep_str: str) -> list[str]:
    """Parse a Depends string into a list of package names."""
    if not dep_str:
        return []
    deps = []
    # Split by comma, strip version constraints and alternatives
    for part in dep_str.split(","):
        part = part.strip()
        # Handle alternatives: "a | b" → just take first
        if "|" in part:
            part = part.split("|")[0].strip()
        # Strip version: "pkg (>= 1.0)"
        part = re.sub(r"\s*\([^)]*\)", "", part)
        part = part.strip()
        if part:
            deps.append(part)
    return deps


def resolve_deps(targets: list[str], all_packages: dict) -> list[str]:
    """Recursively resolve all dependencies. Returns ordered list of package names."""
    resolved = []
    visited = set()
    queue = deque(targets)

    while queue:
        pkg = queue.popleft()
        if pkg in visited:
            continue
        visited.add(pkg)

        if pkg not in all_packages:
            print(f"  WARNING: package not found in repos: {pkg}")
            continue

        resolved.append(pkg)
        info = all_packages[pkg]
        deps = parse_depends(info.get("Depends", ""))
        for dep in deps:
            if dep not in visited:
                queue.append(dep)

    return resolved


def download_file(url: str, dest: str) -> bool:
    """Download a file to dest. Returns True on success."""
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "phase1-downloader/1.0"})
        with urllib.request.urlopen(req, timeout=120, context=_SSL_CTX) as resp:
            with open(dest, "wb") as f:
                f.write(resp.read())
        return True
    except Exception as e:
        print(f"    ERROR: {e}")
        return False


def main():
    os.makedirs(DEB_DIR, exist_ok=True)
    os.makedirs(WHEEL_DIR, exist_ok=True)

    # ---- Step 1: Fetch package indices ----
    print("=" * 60)
    print("Step 1: Fetching package indices...")
    print("=" * 60)

    all_packages = {}
    for repo in REPOS:
        try:
            text = fetch_packages_gz(repo["packages_url"])
            pkgs = parse_packages(text)
            # Add base URL info to each package entry
            for name, info in pkgs.items():
                info["_repo_base"] = repo["base_url"]
            all_packages.update(pkgs)
            print(f"  {repo['name']}: {len(pkgs)} packages")
        except Exception as e:
            print(f"  ERROR fetching {repo['name']}: {e}")

    print(f"\n  Total packages in index: {len(all_packages)}")

    # ---- Step 2: Resolve dependencies ----
    print("\n" + "=" * 60)
    print("Step 2: Resolving dependencies...")
    print("=" * 60)

    all_targets = TARGET_PACKAGES + EXTRA_PACKAGES
    needed = resolve_deps(all_targets, all_packages)

    print(f"  Target packages: {len(all_targets)}")
    print(f"  Total with deps: {len(needed)}")

    # ---- Step 3: Download .deb files ----
    print("\n" + "=" * 60)
    print("Step 3: Downloading .deb packages...")
    print("=" * 60)

    total_size = 0
    downloaded = 0
    failed = []

    for i, pkg_name in enumerate(needed):
        info = all_packages[pkg_name]
        filename = info.get("Filename", "")
        base = info.get("_repo_base", "")
        url = f"{base}/{filename}"
        dest = os.path.join(DEB_DIR, os.path.basename(filename))

        if os.path.exists(dest):
            size = os.path.getsize(dest)
            total_size += size
            downloaded += 1
            continue

        print(f"  [{i+1}/{len(needed)}] {pkg_name} ({os.path.basename(filename)})")
        if download_file(url, dest):
            downloaded += 1
            size = os.path.getsize(dest)
            total_size += size
        else:
            failed.append(pkg_name)

    print(f"\n  Downloaded: {downloaded}/{len(needed)}")
    print(f"  Total size: {total_size / 1024 / 1024:.1f} MB")
    if failed:
        print(f"  Failed: {len(failed)} packages")
        for f in failed:
            print(f"    - {f}")

    # ---- Step 4: Download pip wheels ----
    print("\n" + "=" * 60)
    print("Step 4: Downloading pip wheels...")
    print("=" * 60)

    pip_args = [
        sys.executable, "-m", "pip", "download",
        "--dest", WHEEL_DIR,
        "--platform", "manylinux2014_aarch64",
        "--platform", "linux_aarch64",
        "--python-version", "310",
        "--only-binary", ":all:",
        "--no-deps",
    ] + PIP_PACKAGES

    print(f"  Running: pip download (ARM64 wheels)")
    try:
        subprocess.run(pip_args, check=True)
        wheel_size = sum(
            os.path.getsize(os.path.join(WHEEL_DIR, f))
            for f in os.listdir(WHEEL_DIR)
        )
        print(f"  Pip wheels size: {wheel_size / 1024 / 1024:.1f} MB")
    except subprocess.CalledProcessError as e:
        print(f"  WARNING: pip download failed: {e}")
        print(f"  (pip packages will need to be downloaded manually)")

    # ---- Step 5: Create install script for device ----
    print("\n" + "=" * 60)
    print("Step 5: Creating install script...")
    print("=" * 60)

    install_sh = os.path.join(OUTPUT_DIR, "install_on_device.sh")
    with open(install_sh, "w") as f:
        f.write("#!/bin/bash\n")
        f.write("# Auto-generated install script for Phase 1 dependencies\n")
        f.write("set -e\n\n")
        f.write('echo "Installing .deb packages..."\n')
        f.write("cd $(dirname $0)\n")
        f.write("sudo dpkg -i debs/*.deb 2>&1 || sudo apt-get install -f -y\n")
        f.write('echo ""\n')
        f.write('echo "Installing pip packages..."\n')
        f.write("sudo pip3 install wheels/*.whl 2>/dev/null || pip3 install --user wheels/*.whl\n")
        f.write('echo ""\n')
        f.write('echo "Done! Source ROS2: source /opt/ros/humble/setup.bash"\n')

    os.chmod(install_sh, 0o755)
    print(f"  Created: {install_sh}")

    # ---- Summary ----
    print("\n" + "=" * 60)
    print("DOWNLOAD COMPLETE")
    print("=" * 60)
    print(f"  Directory: {OUTPUT_DIR}/")
    print(f"  .deb files: {downloaded}")
    print(f"  Total size: {total_size / 1024 / 1024:.1f} MB")
    print("")
    print("Next steps:")
    print(f"  1. tar czf phase1_deps.tar.gz {OUTPUT_DIR}/")
    print("  2. scp phase1_deps.tar.gz root@192.168.137.10:/tmp/")
    print("  3. ssh → tar xzf /tmp/phase1_deps.tar.gz")
    print("  4. cd phase1_deps && ./install_on_device.sh")


if __name__ == "__main__":
    main()
