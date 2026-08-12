#!/usr/bin/env python3
"""IMX296 AprilTag landing observer.

This is the first flight-stack boundary: it observes a landing tag and writes
measurements to a CSV file. It does not connect to a flight controller and it
does not send control commands.

The pose is reported in the camera optical frame (x right, y down, z forward).
Converting that pose to the aircraft BODY_FRD frame is intentionally left to a
calibrated mount transform instead of guessing the camera installation.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from picamera2 import Picamera2
from pupil_apriltags import Detector


@dataclass
class Observation:
    timestamp_monotonic: float
    timestamp_utc: str
    tag_id: int
    center_x_px: float
    center_y_px: float
    area_px2: float
    x_m: Optional[float]
    y_m: Optional[float]
    z_m: Optional[float]
    distance_m: Optional[float]
    reprojection_error_px: Optional[float]
    decision_margin: Optional[float]
    hamming: Optional[int]
    valid: bool


class AprilTagObserver:
    def __init__(
        self,
        tag_id: int,
        tag_size_m: float,
        camera_matrix: Optional[np.ndarray],
        distortion: Optional[np.ndarray],
        min_area_px: float,
        min_decision_margin: float,
        max_reprojection_error_px: float,
        range_scale: float = 1.0,
        range_offset_m: float = 0.0,
        detector_threads: int = 2,
        quad_decimate: float = 1.0,
    ) -> None:
        self.tag_id = tag_id
        self.tag_size_m = tag_size_m
        self.camera_matrix = camera_matrix
        self.distortion = distortion
        self.min_area_px = min_area_px
        self.min_decision_margin = min_decision_margin
        self.max_reprojection_error_px = max_reprojection_error_px
        self.range_scale = range_scale
        self.range_offset_m = range_offset_m
        if detector_threads <= 0 or quad_decimate <= 0:
            raise ValueError("detector_threads and quad_decimate must be positive")
        self._undistort_size: Optional[tuple[int, int]] = None
        self._undistort_map1: Optional[np.ndarray] = None
        self._undistort_map2: Optional[np.ndarray] = None
        self._pose_camera_matrix: Optional[np.ndarray] = None

        self.detector = Detector(
            families="tag36h11",
            nthreads=detector_threads,
            quad_decimate=quad_decimate,
            quad_sigma=0.0,
            refine_edges=True,
        )

    def detect(self, frame_bgr: np.ndarray) -> tuple[list[Observation], np.ndarray]:
        estimate_pose = self.camera_matrix is not None and self.distortion is not None
        working_bgr = frame_bgr
        camera_params = None
        if estimate_pose:
            image_size = (frame_bgr.shape[1], frame_bgr.shape[0])
            if self._undistort_size != image_size:
                self._pose_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
                    self.camera_matrix,
                    self.distortion,
                    image_size,
                    0.0,
                    image_size,
                )
                self._undistort_map1, self._undistort_map2 = cv2.initUndistortRectifyMap(
                    self.camera_matrix,
                    self.distortion,
                    None,
                    self._pose_camera_matrix,
                    image_size,
                    cv2.CV_32FC1,
                )
                self._undistort_size = image_size
            working_bgr = cv2.remap(
                frame_bgr,
                self._undistort_map1,
                self._undistort_map2,
                cv2.INTER_LINEAR,
            )
            pose_matrix = self._pose_camera_matrix
            camera_params = (
                float(pose_matrix[0, 0]),
                float(pose_matrix[1, 1]),
                float(pose_matrix[0, 2]),
                float(pose_matrix[1, 2]),
            )
        gray = cv2.cvtColor(working_bgr, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=estimate_pose,
            camera_params=camera_params,
            tag_size=self.tag_size_m,
        )

        annotated = working_bgr.copy()
        observations: list[Observation] = []
        if not detections:
            return observations, annotated

        for detection in detections:
            detected_id = int(detection.tag_id)
            points = np.asarray(detection.corners, dtype=np.float32).reshape(4, 2)
            center = points.mean(axis=0)
            area = abs(float(cv2.contourArea(points)))
            x_m = y_m = z_m = distance_m = reprojection_error = None
            decision_margin = float(detection.decision_margin)
            hamming = int(detection.hamming)
            valid = (
                detected_id == self.tag_id
                and area >= self.min_area_px
                and decision_margin >= self.min_decision_margin
            )

            if estimate_pose:
                translation = np.asarray(detection.pose_t, dtype=float).reshape(3)
                raw_distance_m = float(np.linalg.norm(translation))
                corrected_distance_m = (
                    self.range_scale * raw_distance_m + self.range_offset_m
                )
                if raw_distance_m > 1e-9:
                    translation *= corrected_distance_m / raw_distance_m
                x_m, y_m, z_m = (float(value) for value in translation)
                distance_m = float(np.linalg.norm(translation))
                reprojection_error = float(detection.pose_err)
                valid = valid and reprojection_error <= self.max_reprojection_error_px

            timestamp_monotonic = time.monotonic()
            timestamp_utc = datetime.now(timezone.utc).isoformat()
            observations.append(
                Observation(
                    timestamp_monotonic,
                    timestamp_utc,
                    detected_id,
                    float(center[0]),
                    float(center[1]),
                    area,
                    x_m,
                    y_m,
                    z_m,
                    distance_m,
                    reprojection_error,
                    decision_margin,
                    hamming,
                    valid,
                )
            )

            color = (0, 220, 0) if valid else (0, 0, 255)
            cv2.polylines(annotated, [points.astype(np.int32)], True, color, 2)
            cv2.drawMarker(
                annotated,
                tuple(np.round(center).astype(int)),
                color,
                cv2.MARKER_CROSS,
                24,
                2,
            )
            label = f"id={detected_id} {'VALID' if valid else 'REJECTED'}"
            if distance_m is not None:
                label += f" d={distance_m:.2f}m"
            cv2.putText(
                annotated,
                label,
                tuple(np.round(points[0]).astype(int)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
                cv2.LINE_AA,
            )

        return observations, annotated


def object_points(tag_size_m: float) -> np.ndarray:
    half = tag_size_m / 2.0
    return np.array(
        [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
        dtype=np.float32,
    )


def load_calibration(path: Optional[Path]) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    if path is None:
        return None, None
    if not path.exists():
        raise FileNotFoundError(f"Calibration file does not exist: {path}")
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    try:
        camera_matrix = storage.getNode("camera_matrix").mat()
        distortion = storage.getNode("distortion_coefficients").mat()
    finally:
        storage.release()
    if camera_matrix is None or distortion is None:
        raise ValueError(
            "Calibration must contain camera_matrix and distortion_coefficients"
        )
    return camera_matrix, distortion


def load_range_correction(path: Optional[Path]) -> tuple[float, float]:
    if path is None:
        return 1.0, 0.0
    if not path.exists():
        raise FileNotFoundError(f"Range correction file does not exist: {path}")
    with path.open("r", encoding="utf-8") as correction_file:
        data = json.load(correction_file)
    scale = float(data.get("scale", 1.0))
    offset_m = float(data.get("offset_m", 0.0))
    if scale <= 0:
        raise ValueError("Range correction scale must be positive")
    return scale, offset_m


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="IMX296 AprilTag landing observer")
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--tag-size-m", type=float, default=0.2)
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--range-correction", type=Path)
    parser.add_argument("--image", type=Path, help="detect one offline image instead of opening the camera")
    parser.add_argument("--min-area-px", type=float, default=150.0)
    parser.add_argument("--min-decision-margin", type=float, default=20.0)
    parser.add_argument("--max-reprojection-error-px", type=float, default=2.5)
    parser.add_argument("--width", type=int, default=1456)
    parser.add_argument("--height", type=int, default=1088)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--detector-threads", type=int, default=2)
    parser.add_argument("--quad-decimate", type=float, default=1.0)
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument("--output", type=Path, default=Path.home() / "landing_observer")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--mavlink-dry-run",
        action="store_true",
        help="write MAVLink 2 LANDING_TARGET packets to JSONL; never opens a link",
    )
    return parser


def write_observation(writer: csv.DictWriter, observation: Observation) -> None:
    writer.writerow(observation.__dict__)


def main() -> None:
    args = make_parser().parse_args()
    if args.tag_size_m <= 0 or args.fps <= 0:
        raise ValueError("tag size and FPS must be positive")

    camera_matrix, distortion = load_calibration(args.calibration)
    range_scale, range_offset_m = load_range_correction(args.range_correction)
    if args.range_correction is not None:
        print(
            f"Range correction: scale={range_scale:.9f}, "
            f"offset={range_offset_m:.9f}m"
        )
    if camera_matrix is None:
        print("WARNING: no calibration supplied; pixel detections only, no metric pose")

    args.output.mkdir(parents=True, exist_ok=True)
    csv_path = args.output / "observations.csv"
    image_path = args.output / "latest_annotated.jpg"
    fieldnames = list(Observation.__dataclass_fields__)
    mavlink_file = None
    if args.mavlink_dry_run:
        mavlink_path = args.output / "landing_target_dry_run.jsonl"
        mavlink_file = mavlink_path.open("w", encoding="utf-8")
        print(f"MAVLink dry-run JSONL: {mavlink_path}")

    def write_mavlink_packet(observation: Observation) -> None:
        if mavlink_file is None:
            return
        from mavlink_landing_target import observation_to_packet, pack_message

        packet = observation_to_packet(observation)
        if packet is None:
            return
        record = {
            "message": "LANDING_TARGET",
            "frame": "camera-optical",
            "position_valid": False,
            "packet": packet.as_dict(),
            "mavlink_v2_hex": pack_message(packet).hex(),
        }
        mavlink_file.write(json.dumps(record, ensure_ascii=False) + "\n")
        mavlink_file.flush()

    observer = AprilTagObserver(
        args.tag_id,
        args.tag_size_m,
        camera_matrix,
        distortion,
        args.min_area_px,
        args.min_decision_margin,
        args.max_reprojection_error_px,
        range_scale,
        range_offset_m,
        args.detector_threads,
        args.quad_decimate,
    )

    if args.image is not None:
        frame_bgr = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
        if frame_bgr is None:
            raise FileNotFoundError(f"Could not read image: {args.image}")
        observations, annotated = observer.detect(frame_bgr)
        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            for observation in observations:
                write_observation(writer, observation)
                write_mavlink_packet(observation)
                print(observation)
        cv2.imwrite(str(image_path), annotated)
        if mavlink_file is not None:
            mavlink_file.close()
        print(f"CSV log: {csv_path}")
        print(f"Annotated frame: {image_path}")
        return

    picam2 = Picamera2()
    frame_period_us = int(1_000_000 / args.fps)
    configuration = picam2.create_video_configuration(
        main={"format": "RGB888", "size": (args.width, args.height)},
        controls={"FrameDurationLimits": (frame_period_us, frame_period_us)},
        buffer_count=4,
    )
    picam2.configure(configuration)
    picam2.start()

    start = time.monotonic()
    first_frame = True
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        try:
            while True:
                arrays, _metadata = picam2.capture_arrays(["main"])
                frame = arrays[0]
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                observations, annotated = observer.detect(frame_bgr)
                for observation in observations:
                    write_observation(writer, observation)
                    write_mavlink_packet(observation)
                    print(observation)
                csv_file.flush()
                if first_frame or observations:
                    cv2.imwrite(str(image_path), annotated)
                    first_frame = False

                if args.headless:
                    if args.duration_s <= 0 or time.monotonic() - start >= args.duration_s:
                        break
                else:
                    cv2.imshow("IMX296 AprilTag observer - q quit", annotated)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
                    if args.duration_s > 0 and time.monotonic() - start >= args.duration_s:
                        break
        finally:
            picam2.stop()
            picam2.close()
            cv2.destroyAllWindows()
            if mavlink_file is not None:
                mavlink_file.close()
    print(f"CSV log: {csv_path}")
    print(f"Annotated frame: {image_path}")


if __name__ == "__main__":
    main()
