#!/usr/bin/env python3
"""Automatically collect usable chessboard images for IMX296 calibration."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import cv2
from picamera2 import Picamera2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Collect IMX296 chessboard images")
    parser.add_argument("--output", type=Path, default=Path.home() / "imx296_calibration_images")
    parser.add_argument("--duration-s", type=float, default=180.0)
    parser.add_argument("--target-count", type=int, default=15)
    parser.add_argument("--interval-s", type=float, default=1.5)
    parser.add_argument("--width", type=int, default=1456)
    parser.add_argument("--height", type=int, default=1088)
    parser.add_argument("--corners-cols", type=int, default=9)
    parser.add_argument("--corners-rows", type=int, default=6)
    parser.add_argument(
        "--min-view-change",
        type=float,
        default=1.0,
        help="minimum normalized change from every saved view",
    )
    return parser.parse_args()


def view_signature(corners, cols: int, rows: int, width: int, height: int):
    """Compact signature for position, scale, and tilt of the board."""
    points = corners.reshape(-1, 2)
    top_left = points[0]
    top_right = points[cols - 1]
    bottom_left = points[(rows - 1) * cols]
    center = points.mean(axis=0)
    board_width = math.hypot(*(top_right - top_left)) / width
    board_height = math.hypot(*(bottom_left - top_left)) / height
    angle = math.atan2(float(top_right[1] - top_left[1]), float(top_right[0] - top_left[0]))
    return (
        float(center[0] / width),
        float(center[1] / height),
        float(board_width),
        float(board_height),
        float(angle),
    )


def view_change(a, b) -> float:
    """Return a unitless change score; 1.0 is a meaningful new view."""
    scales = (0.05, 0.05, 0.05, 0.05, math.radians(5.0))
    return math.sqrt(sum(((x - y) / scale) ** 2 for x, y, scale in zip(a, b, scales)))


def main() -> None:
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    pattern = (args.corners_cols, args.corners_rows)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"format": "RGB888", "size": (args.width, args.height)},
        buffer_count=4,
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2.0)

    print(f"Collecting {args.target_count} usable views into {args.output}")
    print("Keep the entire chessboard visible. Slowly move it through different positions and angles.")
    start = time.monotonic()
    last_save = 0.0
    saved = 0
    signatures = []
    try:
        while time.monotonic() - start < args.duration_s and saved < args.target_count:
            frame = picam2.capture_array("main")
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            found, corners = cv2.findChessboardCorners(gray, pattern, flags)
            now = time.monotonic()
            if found and now - last_save >= args.interval_s:
                refined = cv2.cornerSubPix(
                    gray,
                    corners,
                    (11, 11),
                    (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
                )
                signature = view_signature(refined, args.corners_cols, args.corners_rows, args.width, args.height)
                if signatures and max(view_change(signature, old) for old in signatures) < args.min_view_change:
                    time.sleep(0.05)
                    continue
                annotated = frame.copy()
                cv2.drawChessboardCorners(annotated, pattern, refined, found)
                stem = f"calib_{saved:02d}"
                cv2.imwrite(str(args.output / f"{stem}.jpg"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                cv2.imwrite(str(args.output / f"{stem}_corners.jpg"), cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
                saved += 1
                signatures.append(signature)
                last_save = now
                print(f"saved {saved}/{args.target_count}: {stem}.jpg")
            time.sleep(0.05)
    finally:
        picam2.close()

    print(f"Finished: {saved} usable views")
    if saved < 8:
        raise SystemExit("Not enough views; need at least 8, preferably 15-25")


if __name__ == "__main__":
    main()
