"""Publish quality-gated OV9281 observations through MAVROS LANDING_TARGET."""

from __future__ import annotations

import json
import math
import time
import urllib.request
from pathlib import Path

import rclpy
from mavros_msgs.msg import LandingTarget
from rclpy.node import Node
from std_msgs.msg import String

from air_ground_landing.landing_target_bridge import BridgeConfig, LandingTargetBridge
from air_ground_landing.mavros_frames import body_frd_pose_to_ros_baselink


def capture_evidence_metadata(status, observation, now_s, previous_sensor_ns=None):
    """Qualify optional camera metadata without changing legacy target outputs.

    The bridge's older capture_time_s is derived from analysis-completion age.
    It is explicitly labelled and cannot qualify vertical slow evidence.
    """
    sequence = observation.source_sequence
    derived = observation.capture_time_s
    result = dict(accepted_capture_time_s=derived,
        accepted_time_basis="derived_analysis_time", accepted_capture_timing_valid=False,
        accepted_capture_timing_reason="NO_CAMERA_CAPTURE_METADATA",
        accepted_sensor_timestamp_ns=None)
    if "capture_timing_valid" not in status:
        return result
    result.update(accepted_capture_time_s=None, accepted_time_basis="libcamera_sensor_timestamp",
                  accepted_capture_timing_reason="CAPTURE_METADATA_UNVERIFIED")
    stamp = status.get("capture_monotonic_s")
    sensor_ns = status.get("capture_sensor_timestamp_ns")
    identifiers = (sequence, status.get("analysis_sequence"), status.get("capture_analysis_sequence"))
    if (status.get("capture_timing_valid") is not True
            or status.get("capture_timestamp_source") != "libcamera_sensor_timestamp"
            or status.get("capture_clock_id") != "CLOCK_MONOTONIC"
            or status.get("capture_sensor_clock_id") != "CLOCK_BOOTTIME"
            or status.get("capture_time_semantics") != "RPI_FIRST_PIXEL_EXPOSURE_START"
            or not all(isinstance(value, int) and not isinstance(value, bool) and value >= 0 for value in identifiers)
            or len(set(identifiers)) != 1
            or not isinstance(stamp, (int, float)) or isinstance(stamp, bool)
            or not math.isfinite(stamp) or not 0 <= now_s-stamp <= .35
            or not isinstance(sensor_ns, int) or isinstance(sensor_ns, bool) or sensor_ns <= 0
            or previous_sensor_ns is not None and sensor_ns <= previous_sensor_ns):
        return result
    result.update(accepted_capture_time_s=stamp, accepted_capture_timing_valid=True,
                  accepted_capture_timing_reason="CAPTURE_METADATA_VALID",
                  accepted_sensor_timestamp_ns=sensor_ns)
    return result


class LandingTargetAdapter(Node):
    def __init__(self) -> None:
        super().__init__("landing_target_adapter")
        defaults = {
            "config_path": "",
            "status_url": "http://127.0.0.1:8765/api/vision/status",
            "http_timeout_s": 0.15,
            "poll_rate_hz": 10.0,
            "environment": "offline",
            "flight_use_approved": False,
            "allow_landing_target_output": False,
            "preview_topic": "/landing/landing_target/preview",
            "status_topic": "/landing/landing_target/status",
            "mavros_output_topic": "/mavros/landing_target/raw",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        config_path = Path(str(self.get_parameter("config_path").value))
        if not config_path.is_file():
            raise ValueError("landing target config_path must be an existing file")
        root = json.loads(config_path.read_text(encoding="utf-8"))
        self.bridge = LandingTargetBridge(BridgeConfig.from_mapping(root))
        environment = str(self.get_parameter("environment").value).lower()
        if environment not in ("offline", "sitl", "hardware"):
            raise ValueError("environment must be offline, sitl or hardware")
        approved = environment == "sitl" or (
            environment == "hardware"
            and bool(self.get_parameter("flight_use_approved").value)
        )
        self.output_enabled = bool(
            self.get_parameter("allow_landing_target_output").value
        ) and approved
        self.status_url = str(self.get_parameter("status_url").value)
        self.http_timeout_s = float(self.get_parameter("http_timeout_s").value)
        rate_hz = float(self.get_parameter("poll_rate_hz").value)
        if self.http_timeout_s <= 0.0 or not 1.0 <= rate_hz <= 50.0:
            raise ValueError("invalid landing target HTTP/rate configuration")
        self.last_accepted_s: float | None = None
        self.last_capture_sensor_ns = None
        self.preview_publisher = self.create_publisher(
            LandingTarget, str(self.get_parameter("preview_topic").value), 10
        )
        self.actual_publisher = self.create_publisher(
            LandingTarget, str(self.get_parameter("mavros_output_topic").value), 10
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.create_timer(1.0 / rate_hz, self._tick)

    @staticmethod
    def _now_s() -> float:
        return time.monotonic()

    def _message(self, packet) -> LandingTarget:
        position_ros_flu, orientation_ros_flu = body_frd_pose_to_ros_baselink(
            (packet.x, packet.y, packet.z),
            packet.q,
        )
        message = LandingTarget()
        message.header.stamp = self.get_clock().now().to_msg()
        message.target_num = int(packet.target_num)
        message.frame = int(packet.frame)
        message.angle = [float(packet.angle_x), float(packet.angle_y)]
        message.distance = float(packet.distance)
        message.size = [float(packet.size_x), float(packet.size_y)]
        message.pose.position.x = position_ros_flu[0]
        message.pose.position.y = position_ros_flu[1]
        message.pose.position.z = position_ros_flu[2]
        message.pose.orientation.w = orientation_ros_flu[0]
        message.pose.orientation.x = orientation_ros_flu[1]
        message.pose.orientation.y = orientation_ros_flu[2]
        message.pose.orientation.z = orientation_ros_flu[3]
        message.type = int(packet.type)
        return message

    def _tick(self) -> None:
        now_s = self._now_s()
        accepted = False
        transmitted = False
        reason = "NO_SAMPLE"
        accepted_target_num = None
        accepted_distance_m = None
        accepted_vertical_distance_m = None
        accepted_frame_id = None
        accepted_body_frd_m = None
        capture_metadata = dict(accepted_capture_time_s=None, accepted_time_basis="unavailable",
            accepted_capture_timing_valid=False, accepted_capture_timing_reason="NO_ACCEPTED_FRAME",
            accepted_sensor_timestamp_ns=None)
        try:
            with urllib.request.urlopen(self.status_url, timeout=self.http_timeout_s) as response:
                status = json.loads(response.read().decode("utf-8"))
            result = self.bridge.process_status(
                status,
                received_time_s=now_s,
                wall_time_usec=time.time_ns() // 1_000,
            )
            reason = result.reason
            if result.packet is not None:
                if result.observation is not None:
                    accepted_frame_id = result.observation.source_sequence
                    capture_metadata = capture_evidence_metadata(status, result.observation,
                        self._now_s(), self.last_capture_sensor_ns)
                    if capture_metadata["accepted_capture_timing_valid"]:
                        self.last_capture_sensor_ns = capture_metadata["accepted_sensor_timestamp_ns"]
                accepted_target_num = int(result.packet.target_num)
                accepted_distance_m = float(result.packet.distance)
                accepted_vertical_distance_m = abs(float(result.packet.z))
                accepted_body_frd_m = [float(result.packet.x), float(result.packet.y), float(result.packet.z)]
                message = self._message(result.packet)
                self.preview_publisher.publish(message)
                accepted = True
                self.last_accepted_s = now_s
                if self.output_enabled:
                    self.actual_publisher.publish(message)
                    transmitted = True
        except Exception as exc:
            reason = f"STATUS_ERROR:{type(exc).__name__}:{exc}"
        age_s = None if self.last_accepted_s is None else now_s - self.last_accepted_s
        stream_healthy = bool(
            age_s is not None and age_s <= self.bridge.config.loss_timeout_s
        )
        status_message = String()
        status_message.data = json.dumps(
            {
                "node": "LANDING_TARGET_ADAPTER_ROS2",
                "accepted_this_poll": accepted,
                "reason": reason,
                "stream_healthy": stream_healthy,
                "last_accepted_age_s": age_s,
                "output_enabled": self.output_enabled,
                "mavlink_transmitted": transmitted,
                "accepted_target_num": accepted_target_num,
                "accepted_distance_m": accepted_distance_m,
                "accepted_vertical_distance_m": accepted_vertical_distance_m,
                "accepted_frame_id": accepted_frame_id,
                "accepted_body_frd_m": accepted_body_frd_m,
                "accepted_body_frame": "BODY_FRD" if accepted_body_frd_m is not None else None,
                "accepted_vertical_distance_basis": "BODY_FRD_DOWN_COMPONENT",
                **capture_metadata,
            },
            separators=(",", ":"),
        )
        self.status_publisher.publish(status_message)


def main() -> None:
    rclpy.init()
    node = LandingTargetAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
