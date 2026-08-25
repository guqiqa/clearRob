#!/usr/bin/env python3
"""Offline stereo calibration for synchronized left/right image directories.

The tool deliberately writes the OpenCV keys consumed by stereo_depth_node.py.
Images are paired by identical filename, or by sorted order when --allow-order
is explicitly supplied.
"""

from __future__ import annotations

import argparse
import glob
import os
from pathlib import Path

IMAGE_EXTENSIONS = ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.pgm")


def image_paths(directory: Path) -> list[Path]:
    paths: list[Path] = []
    for pattern in IMAGE_EXTENSIONS:
        paths.extend(Path(p) for p in glob.glob(str(directory / pattern)))
    return sorted(set(paths), key=lambda p: p.name)


def pair_images(left_dir: Path, right_dir: Path, allow_order: bool) -> list[tuple[Path, Path]]:
    left = image_paths(left_dir)
    right = image_paths(right_dir)
    if not left or not right:
        raise RuntimeError("both image directories must contain PNG/JPEG/BMP/PGM files")
    right_by_name = {p.name: p for p in right}
    pairs = [(p, right_by_name[p.name]) for p in left if p.name in right_by_name]
    if len(pairs) < 3 and allow_order:
        count = min(len(left), len(right))
        pairs = list(zip(left[:count], right[:count]))
    if len(pairs) < 3:
        raise RuntimeError(
            f"only {len(pairs)} matching pairs found; use identical names or --allow-order"
        )
    return pairs


def find_corners(gray: np.ndarray, pattern: tuple[int, int]) -> np.ndarray | None:
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def reprojection_error(
    object_points: list[np.ndarray], image_points: list[np.ndarray],
    rvecs: list[np.ndarray], tvecs: list[np.ndarray], camera: np.ndarray, distortion: np.ndarray,
) -> float:
    total = 0.0
    count = 0
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera, distortion)
        total += float(cv2.norm(img, projected, cv2.NORM_L2)) ** 2
        count += len(obj)
    return float(np.sqrt(total / max(count, 1)))


def write_matrix(fs: cv2.FileStorage, name: str, value: np.ndarray) -> None:
    fs.write(name, np.asarray(value, dtype=np.float64))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-dir", required=True, type=Path)
    parser.add_argument("--right-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--cols", type=int, default=7, help="inner corners across the board")
    parser.add_argument("--rows", type=int, default=10, help="inner corners down the board")
    parser.add_argument("--square-size-m", type=float, default=0.02)
    parser.add_argument("--allow-order", action="store_true")
    parser.add_argument("--min-pairs", type=int, default=20)
    args = parser.parse_args()

    try:
        global cv2, np
        import cv2  # type: ignore
        import numpy as np  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "stereo calibration requires Python OpenCV and NumPy; "
            "install python3-opencv/python3-numpy in the calibration environment"
        ) from exc

    if args.cols < 2 or args.rows < 2 or args.square_size_m <= 0:
        parser.error("cols/rows must be >= 2 and square-size-m must be positive")
    pairs = pair_images(args.left_dir, args.right_dir, args.allow_order)
    if len(pairs) < args.min_pairs:
        raise RuntimeError(f"{len(pairs)} pairs found, but --min-pairs requires {args.min_pairs}")

    pattern = (args.cols, args.rows)
    board = np.zeros((args.cols * args.rows, 3), np.float32)
    board[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    board *= float(args.square_size_m)

    object_points: list[np.ndarray] = []
    left_points: list[np.ndarray] = []
    right_points: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None
    rejected = 0
    for left_path, right_path in pairs:
        left = cv2.imread(str(left_path), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(right_path), cv2.IMREAD_GRAYSCALE)
        if left is None or right is None or left.shape != right.shape:
            rejected += 1
            continue
        current_size = (left.shape[1], left.shape[0])
        if image_size is None:
            image_size = current_size
        if current_size != image_size:
            rejected += 1
            continue
        left_corners = find_corners(left, pattern)
        right_corners = find_corners(right, pattern)
        if left_corners is None or right_corners is None:
            rejected += 1
            continue
        object_points.append(board.copy())
        left_points.append(left_corners)
        right_points.append(right_corners)

    if image_size is None or len(object_points) < args.min_pairs:
        raise RuntimeError(
            f"only {len(object_points)} valid checkerboard pairs; rejected={rejected}, "
            f"need at least {args.min_pairs}"
        )

    mono_flags = cv2.CALIB_RATIONAL_MODEL
    rms_left, K_left, D_left, rvec_l, tvec_l = cv2.calibrateCamera(
        object_points, left_points, image_size, None, None, flags=mono_flags
    )
    rms_right, K_right, D_right, rvec_r, tvec_r = cv2.calibrateCamera(
        object_points, right_points, image_size, None, None, flags=mono_flags
    )
    stereo_flags = cv2.CALIB_FIX_INTRINSIC
    rms_stereo, K_left, D_left, K_right, D_right, R, T, E, F = cv2.stereoCalibrate(
        object_points, left_points, right_points, K_left, D_left, K_right, D_right,
        image_size, criteria=(cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-6),
        flags=stereo_flags,
    )
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K_left, D_left, K_right, D_right, image_size, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0,
    )
    baseline = float(np.linalg.norm(T.reshape(-1)))
    left_error = reprojection_error(object_points, left_points, rvec_l, tvec_l, K_left, D_left)
    right_error = reprojection_error(object_points, right_points, rvec_r, tvec_r, K_right, D_right)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(args.output), cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened():
        raise RuntimeError(f"cannot write {args.output}")
    fs.write("image_width", image_size[0])
    fs.write("image_height", image_size[1])
    fs.write("square_size_m", float(args.square_size_m))
    fs.write("checkerboard_cols", args.cols)
    fs.write("checkerboard_rows", args.rows)
    fs.write("valid_pairs", len(object_points))
    fs.write("rejected_pairs", rejected)
    fs.write("rms_left", float(rms_left))
    fs.write("rms_right", float(rms_right))
    fs.write("rms_stereo", float(rms_stereo))
    fs.write("reprojection_error_left", left_error)
    fs.write("reprojection_error_right", right_error)
    fs.write("baseline_m", baseline)
    for name, value in (("K_left", K_left), ("D_left", D_left), ("K_right", K_right),
                        ("D_right", D_right), ("R", R), ("T", T), ("E", E), ("F", F),
                        ("R1", R1), ("R2", R2), ("P1", P1), ("P2", P2), ("Q", Q)):
        write_matrix(fs, name, value)
    fs.release()
    print(
        f"valid_pairs={len(object_points)} rejected={rejected} image={image_size[0]}x{image_size[1]} "
        f"rms_stereo={rms_stereo:.4f} baseline_m={baseline:.6f} "
        f"reprojection_left={left_error:.4f} reprojection_right={right_error:.4f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
