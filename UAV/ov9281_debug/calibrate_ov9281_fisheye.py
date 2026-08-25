#!/usr/bin/env python3
"""Calibrate a wide-angle OV9281 lens with OpenCV's fisheye model."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def detect_views(images: Path, cols: int, rows: int, square_m: float):
    template = np.zeros((1, cols * rows, 3), np.float64)
    template[0, :, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    template *= square_m
    views = []
    size = None
    for path in sorted(images.glob("calib_[0-9][0-9].jpg")):
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        this_size = (gray.shape[1], gray.shape[0])
        if size is not None and this_size != size:
            raise ValueError(f"Mixed image sizes: {size} and {this_size}")
        size = this_size
        found, corners = cv2.findChessboardCorners(
            gray, (cols, rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if not found:
            continue
        refined = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4),
        ).reshape(1, -1, 2).astype(np.float64)
        views.append((path, template.copy(), refined))
    if size is None or len(views) < 8:
        raise RuntimeError(f"Need at least 8 usable views, found {len(views)}")
    return size, views


def fit(size, views):
    k = np.array([[size[0] * 0.55, 0.0, size[0] / 2],
                  [0.0, size[0] * 0.55, size[1] / 2],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    d = np.zeros((4, 1), dtype=np.float64)
    flags = (
        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
        | cv2.fisheye.CALIB_CHECK_COND
        | cv2.fisheye.CALIB_FIX_SKEW
        | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS
    )
    rms, k, d, rvecs, tvecs = cv2.fisheye.calibrate(
        [v[1] for v in views], [v[2] for v in views], size, k, d,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-8),
    )
    errors = []
    for view, rvec, tvec in zip(views, rvecs, tvecs):
        projected, _ = cv2.fisheye.projectPoints(view[1], rvec, tvec, k, d)
        residual = projected.reshape(-1, 2) - view[2].reshape(-1, 2)
        errors.append(float(np.sqrt(np.mean(np.sum(residual * residual, axis=1)))))
    return float(rms), k, d, errors


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--images", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--corners-cols", type=int, default=9)
    p.add_argument("--corners-rows", type=int, default=6)
    p.add_argument("--square-size-m", type=float, default=0.017)
    p.add_argument("--min-views", type=int, default=12)
    p.add_argument("--max-view-rms-px", type=float, default=1.5)
    args = p.parse_args()

    size, all_views = detect_views(
        args.images, args.corners_cols, args.corners_rows, args.square_size_m
    )
    active = list(all_views)
    rejected = []
    while True:
        rms, k, d, errors = fit(size, active)
        worst = int(np.argmax(errors))
        if errors[worst] <= args.max_view_rms_px or len(active) <= args.min_views:
            break
        rejected.append((active[worst][0].name, errors[worst]))
        del active[worst]

    mean_error = float(np.mean(errors))
    max_error = float(np.max(errors))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(args.output), cv2.FILE_STORAGE_WRITE)
    try:
        fs.write("camera_model", "fisheye")
        fs.write("camera_matrix", k)
        fs.write("distortion_coefficients", d)
        fs.write("image_width", size[0])
        fs.write("image_height", size[1])
        fs.write("checkerboard_corners_cols", args.corners_cols)
        fs.write("checkerboard_corners_rows", args.corners_rows)
        fs.write("checkerboard_square_size_m", args.square_size_m)
        fs.write("rms_reprojection_error_px", rms)
        fs.write("mean_view_rms_px", mean_error)
        fs.write("max_view_rms_px", max_error)
        fs.write("usable_images", len(active))
        fs.write("rejected_images", len(rejected))
    finally:
        fs.release()

    print(f"usable_images={len(active)} rejected_images={len(rejected)}")
    for name, error in rejected:
        print(f"rejected={name} view_rms_px={error:.4f}")
    print(f"image_size={size[0]}x{size[1]}")
    print(f"rms_reprojection_error_px={rms:.4f}")
    print(f"mean_view_rms_px={mean_error:.4f}")
    print(f"max_view_rms_px={max_error:.4f}")
    print(f"output={args.output}")


if __name__ == "__main__":
    main()
