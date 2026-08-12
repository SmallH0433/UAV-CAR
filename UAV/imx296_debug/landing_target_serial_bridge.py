#!/usr/bin/env python3
"""Safe, disarmed serial bridge for AprilTag LANDING_TARGET messages.

This bridge sends only MAVLink LANDING_TARGET sensor messages. It refuses to
start if the flight controller reports armed, and it never sends arm, mode,
takeoff, landing, or other vehicle-control commands.
"""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
from picamera2 import Picamera2
from pymavlink import mavutil

from landing_observer import AprilTagObserver, load_calibration, load_range_correction
from mavlink_landing_target import (
    MAV_FRAME_CAMERA_OPTICAL,
    MAV_FRAME_BODY_FRD,
    load_body_extrinsics,
    make_message,
    observation_to_packet,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Disarmed LANDING_TARGET serial bridge")
    parser.add_argument("--serial", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--tag-size-m", type=float, default=0.135)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--range-correction", type=Path, required=True)
    parser.add_argument("--width", type=int, default=1456)
    parser.add_argument("--height", type=int, default=1088)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--detector-threads", type=int, default=2)
    parser.add_argument("--quad-decimate", type=float, default=1.0)
    parser.add_argument("--duration-s", type=float, default=15.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--annotated-output",
        type=Path,
        help="optional latest annotated camera frame for bench verification",
    )
    parser.add_argument("--frame", choices=("camera-optical", "body-frd"), default="camera-optical")
    parser.add_argument("--camera-yaw", choices=("nose-left",), help="required for body-frd conversion")
    parser.add_argument("--extrinsics", type=Path, help="camera-to-body JSON; required for body-frd")
    parser.add_argument(
        "--plnd-profile",
        choices=("disabled", "mavlink-enabled"),
        default="disabled",
        help="required real-FC PLND state for a disarmed BODY_FRD bench",
    )
    parser.add_argument(
        "--csv-replay",
        type=Path,
        help="replay previously captured valid observations instead of opening the camera",
    )
    return parser.parse_args()


def armed(heartbeat) -> bool:
    return bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def wait_for_vehicle_heartbeat(link, timeout_s: float = 5.0):
    """Wait for the autopilot heartbeat, ignoring companion/GCS heartbeats."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if message is None:
            continue
        if (
            message.get_srcSystem() == 1
            and message.get_srcComponent() == 1
            and message.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
        ):
            return message
    return None


def normalized_param_id(message) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="ignore")
    return str(value).rstrip("\x00")


def read_param_as_gcs(link, name: str, timeout_s: float = 3.0):
    original_system = link.mav.srcSystem
    original_component = link.mav.srcComponent
    try:
        link.mav.srcSystem = 255
        link.mav.srcComponent = 191
        link.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        for _attempt in range(4):
            link.mav.param_request_read_send(1, 1, name.encode("ascii"), -1)
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                message = link.recv_match(
                    type="PARAM_VALUE", blocking=True, timeout=0.5
                )
                if message is not None and normalized_param_id(message) == name:
                    return float(message.param_value)
        return None
    finally:
        link.mav.srcSystem = original_system
        link.mav.srcComponent = original_component


def row_to_observation(row: dict[str, str]) -> SimpleNamespace:
    def optional_float(name: str):
        value = row.get(name, "")
        return None if value in ("", "None") else float(value)

    return SimpleNamespace(
        valid=row.get("valid", "False").lower() == "true",
        x_m=optional_float("x_m"),
        y_m=optional_float("y_m"),
        z_m=optional_float("z_m"),
        distance_m=optional_float("distance_m"),
        tag_id=int(row.get("tag_id", 0)),
        decision_margin=optional_float("decision_margin"),
        hamming=None,
        timestamp_utc=row.get("timestamp_utc", ""),
    )


def main() -> None:
    args = parse_args()
    extrinsics = None
    if args.frame == "body-frd":
        if args.extrinsics is None:
            raise RuntimeError("Safety stop: real-serial body-frd requires --extrinsics")
        extrinsics = load_body_extrinsics(args.extrinsics)
        if extrinsics.allowed_scope not in (
            "disarmed_plnd_disabled_bench_and_sitl",
            "disarmed_bench_and_sitl",
        ):
            raise RuntimeError("Safety stop: extrinsics are not approved for this bench scope")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    camera_matrix, distortion = load_calibration(args.calibration)
    range_scale, range_offset_m = load_range_correction(args.range_correction)
    observer = AprilTagObserver(
        args.tag_id,
        args.tag_size_m,
        camera_matrix,
        distortion,
        min_area_px=150.0,
        min_decision_margin=20.0,
        max_reprojection_error_px=2.5,
        range_scale=range_scale,
        range_offset_m=range_offset_m,
        detector_threads=args.detector_threads,
        quad_decimate=args.quad_decimate,
    )

    print(f"Opening flight-controller telemetry: {args.serial} @ {args.baud}")
    link = mavutil.mavlink_connection(
        args.serial,
        baud=args.baud,
        source_system=191,
        source_component=191,
        autoreconnect=False,
    )
    heartbeat = wait_for_vehicle_heartbeat(link, timeout_s=5.0)
    if heartbeat is None:
        raise RuntimeError("No flight-controller heartbeat received")
    print(
        f"Heartbeat received: sys={heartbeat.get_srcSystem()} "
        f"comp={heartbeat.get_srcComponent()} armed={armed(heartbeat)}"
    )
    if armed(heartbeat):
        raise RuntimeError("Safety stop: flight controller is armed")
    if args.frame == "body-frd":
        plnd_enabled = read_param_as_gcs(link, "PLND_ENABLED")
        plnd_type = read_param_as_gcs(link, "PLND_TYPE")
        print(f"Bench gate: PLND_ENABLED={plnd_enabled} PLND_TYPE={plnd_type}")
        expected = (0.0, 0.0) if args.plnd_profile == "disabled" else (1.0, 1.0)
        if (plnd_enabled, plnd_type) != expected:
            raise RuntimeError(
                "Safety stop: BODY_FRD bench PLND state does not match "
                f"profile {args.plnd_profile}; expected {expected}"
            )

    replay = None
    if args.csv_replay is not None:
        with args.csv_replay.open("r", newline="", encoding="utf-8") as replay_file:
            replay = [
                row_to_observation(row)
                for row in csv.DictReader(replay_file)
                if row.get("valid", "False").lower() == "true"
            ]
        if not replay:
            raise RuntimeError(f"No valid observations in replay CSV: {args.csv_replay}")

    camera = None
    if replay is None:
        camera = Picamera2()
        frame_period_us = int(1_000_000 / args.fps)
        camera.configure(
            camera.create_video_configuration(
                main={"size": (args.width, args.height), "format": "RGB888"},
                controls={"FrameDurationLimits": (frame_period_us, frame_period_us)},
                buffer_count=4,
            )
        )
        camera.start()
    started = time.monotonic()
    sent = 0
    valid = 0
    received_types: dict[str, int] = {}
    latest_annotated = None
    try:
        with args.output.open("w", encoding="utf-8") as log_file:
            replay_index = 0
            while time.monotonic() - started < args.duration_s:
                while True:
                    incoming = link.recv_match(blocking=False)
                    if incoming is None:
                        break
                    msg_type = incoming.get_type()
                    received_types[msg_type] = received_types.get(msg_type, 0) + 1
                    if (
                        msg_type == "HEARTBEAT"
                        and incoming.get_srcSystem() == heartbeat.get_srcSystem()
                        and incoming.get_srcComponent() == heartbeat.get_srcComponent()
                        and armed(incoming)
                    ):
                        raise RuntimeError("Safety stop: flight controller became armed")

                if replay is not None:
                    observations = [replay[replay_index % len(replay)]]
                    replay_index += 1
                else:
                    frame_rgb = camera.capture_array("main")
                    frame_bgr = frame_rgb[:, :, ::-1].copy()
                    observations, latest_annotated = observer.detect(frame_bgr)
                for observation in observations:
                    if not observation.valid:
                        continue
                    send_observation = observation
                    frame = MAV_FRAME_CAMERA_OPTICAL
                    position_valid = 0
                    if args.frame == "body-frd":
                        send_observation = SimpleNamespace(**vars(observation))
                        (
                            send_observation.x_m,
                            send_observation.y_m,
                            send_observation.z_m,
                        ) = extrinsics.transform(
                            observation.x_m,
                            observation.y_m,
                            observation.z_m,
                        )
                        frame = MAV_FRAME_BODY_FRD
                        position_valid = 1
                    packet = observation_to_packet(
                        send_observation,
                        target_num=0,
                        frame=frame,
                        position_valid=position_valid,
                    )
                    if packet is None:
                        continue
                    link.mav.send(make_message(packet))
                    sent += 1
                    valid += 1
                    log_file.write(
                        json.dumps(
                            {
                                "timestamp_utc": observation.timestamp_utc,
                                "message": "LANDING_TARGET",
                                "frame": args.frame,
                                "position_valid": bool(position_valid),
                                "tag_id": observation.tag_id,
                                "decision_margin": observation.decision_margin,
                                "hamming": observation.hamming,
                                "extrinsics": str(args.extrinsics) if args.extrinsics else None,
                                "packet": packet.as_dict(),
                            },
                            ensure_ascii=False,
                        )
                        + "\n"
                    )
                    log_file.flush()
                if replay is not None:
                    time.sleep(max(0.0, 1.0 / args.fps))
    finally:
        if args.annotated_output is not None and latest_annotated is not None:
            args.annotated_output.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(args.annotated_output), latest_annotated)
        if camera is not None:
            camera.stop()
        link.close()

    print(f"valid_detections={valid} landing_target_sent={sent}")
    print(f"received_message_types={json.dumps(received_types, ensure_ascii=False)}")
    print(f"log={args.output}")
    if args.annotated_output is not None:
        print(f"annotated={args.annotated_output}")
    print("Safety: disarmed sensor messages only; no flight-control command was sent.")


if __name__ == "__main__":
    main()
