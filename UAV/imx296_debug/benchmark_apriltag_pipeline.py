#!/usr/bin/env python3
"""Offline AprilTag throughput benchmark using one recorded full-resolution frame."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
from pupil_apriltags import Detector

from landing_observer import load_calibration, load_range_correction


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Benchmark AprilTag detector settings")
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--range-correction", type=Path, required=True)
    parser.add_argument("--tag-size-m", type=float, default=0.135)
    parser.add_argument("--loops", type=int, default=30)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    frame = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if frame is None:
        raise FileNotFoundError(args.image)
    camera_matrix, distortion = load_calibration(args.calibration)
    scale, offset_m = load_range_correction(args.range_correction)
    image_size = (frame.shape[1], frame.shape[0])
    pose_matrix, _ = cv2.getOptimalNewCameraMatrix(
        camera_matrix, distortion, image_size, 0.0, image_size
    )
    map1, map2 = cv2.initUndistortRectifyMap(
        camera_matrix,
        distortion,
        None,
        pose_matrix,
        image_size,
        cv2.CV_32FC1,
    )
    camera_params = (
        float(pose_matrix[0, 0]),
        float(pose_matrix[1, 1]),
        float(pose_matrix[0, 2]),
        float(pose_matrix[1, 2]),
    )

    results = []
    for threads in (2, 4):
        for decimate in (1.0, 1.5, 2.0, 2.5, 3.0):
            detector = Detector(
                families="tag36h11",
                nthreads=threads,
                quad_decimate=decimate,
                quad_sigma=0.0,
                refine_edges=True,
            )
            distances = []
            margins = []
            valid = 0
            started = time.perf_counter()
            for _ in range(args.loops):
                undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
                gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
                detections = detector.detect(
                    gray,
                    estimate_tag_pose=True,
                    camera_params=camera_params,
                    tag_size=args.tag_size_m,
                )
                matching = [detection for detection in detections if int(detection.tag_id) == 0]
                if not matching:
                    continue
                detection = matching[0]
                raw_distance = float(np.linalg.norm(np.asarray(detection.pose_t).reshape(3)))
                distances.append(scale * raw_distance + offset_m)
                margins.append(float(detection.decision_margin))
                valid += 1
            elapsed = time.perf_counter() - started
            result = {
                "threads": threads,
                "quad_decimate": decimate,
                "loops": args.loops,
                "valid": valid,
                "pipeline_hz": args.loops / elapsed,
                "distance_mean_m": None if not distances else sum(distances) / len(distances),
                "decision_margin_mean": None if not margins else sum(margins) / len(margins),
            }
            results.append(result)
            print(json.dumps(result, ensure_ascii=False))

    baseline = next(
        result for result in results if result["threads"] == 2 and result["quad_decimate"] == 1.0
    )
    baseline_distance = baseline["distance_mean_m"]
    for result in results:
        result["distance_delta_from_baseline_m"] = (
            None
            if result["distance_mean_m"] is None or baseline_distance is None
            else result["distance_mean_m"] - baseline_distance
        )
    args.output.write_text(json.dumps(results, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"output={args.output}")
    print("safety=offline_image_only_no_camera_no_serial")


if __name__ == "__main__":
    main()
