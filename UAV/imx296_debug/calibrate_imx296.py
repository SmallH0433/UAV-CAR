#!/usr/bin/env python3
"""Calibrate IMX296 intrinsics from chessboard images.

The generated YAML is consumed by landing_observer.py. Images must be taken
at the same resolution used by the observer (1456x1088 by default).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate IMX296 intrinsics")
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--corners-cols", type=int, default=9)
    parser.add_argument("--corners-rows", type=int, default=6)
    parser.add_argument("--square-size-m", type=float, default=0.025)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.images.is_dir():
        raise NotADirectoryError(args.images)
    if args.corners_cols < 3 or args.corners_rows < 3:
        raise ValueError("checkerboard dimensions must be inner-corner counts")
    if args.square_size_m <= 0:
        raise ValueError("square size must be positive")

    pattern = (args.corners_cols, args.corners_rows)
    template = np.zeros((args.corners_cols * args.corners_rows, 3), np.float32)
    template[:, :2] = np.mgrid[0 : args.corners_cols, 0 : args.corners_rows].T.reshape(-1, 2)
    template *= args.square_size_m

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    image_size = None
    used = 0
    rejected = 0

    for path in sorted(args.images.glob("*")):
        if path.suffix.lower() not in {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}:
            continue
        # The collector stores *_corners.jpg for visual QA. Those annotated
        # copies must not be used as independent calibration samples.
        if path.stem.endswith("_corners"):
            continue
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            rejected += 1
            continue
        image_size = (image.shape[1], image.shape[0])
        found, corners = cv2.findChessboardCorners(
            image,
            pattern,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH
            | cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_FAST_CHECK,
        )
        if not found:
            rejected += 1
            continue
        refined = cv2.cornerSubPix(
            image,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3),
        )
        object_points.append(template.copy())
        image_points.append(refined)
        used += 1

    if image_size is None or used < 8:
        raise RuntimeError(f"Need at least 8 usable images; found {used}, rejected {rejected}")

    rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    errors = []
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, distortion)
        errors.append(float(np.mean(np.linalg.norm(projected.reshape(-1, 2) - img.reshape(-1, 2), axis=1))))
    mean_error = float(np.mean(errors))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    storage = cv2.FileStorage(str(args.output), cv2.FILE_STORAGE_WRITE)
    try:
        storage.write("camera_matrix", camera_matrix)
        storage.write("distortion_coefficients", distortion)
        storage.write("image_width", image_size[0])
        storage.write("image_height", image_size[1])
        storage.write("checkerboard_corners_cols", args.corners_cols)
        storage.write("checkerboard_corners_rows", args.corners_rows)
        storage.write("checkerboard_square_size_m", args.square_size_m)
        storage.write("rms_reprojection_error_px", rms)
        storage.write("mean_reprojection_error_px", mean_error)
    finally:
        storage.release()

    print(f"usable_images={used} rejected_images={rejected}")
    print(f"image_size={image_size[0]}x{image_size[1]}")
    print(f"rms_reprojection_error_px={rms:.4f}")
    print(f"mean_reprojection_error_px={mean_error:.4f}")
    print(f"output={args.output}")


if __name__ == "__main__":
    main()
