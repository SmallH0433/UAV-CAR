"""Fuse 3D lidar, OV9281 stereo depth and downward ToF for UAV avoidance."""

import json
import math
import struct
import time
from typing import Iterable, Tuple

from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import String

from .perception_math import (
    DIRECTIONS,
    ObstacleSummary,
    combine_summaries,
    scan_to_points,
    summarize_points,
)
from .runtime_timing import create_steady_timer


class RateTracker:
    def __init__(self) -> None:
        self.last = 0.0
        self.last_wall = 0.0
        self.previous = 0.0
        self.filtered_rate = 0.0
        self.count = 0

    def mark(self, now_s: float | None = None) -> None:
        wall_now = time.monotonic()
        now = wall_now if now_s is None else float(now_s)
        self.previous, self.last = self.last, now
        self.last_wall = wall_now
        self.count += 1
        if self.previous > 0.0 and now > self.previous:
            instant = 1.0 / (now - self.previous)
            self.filtered_rate = instant if self.filtered_rate == 0.0 else (
                0.8 * self.filtered_rate + 0.2 * instant
            )

    def report(
        self,
        stale_after: float,
        *,
        now_s: float | None = None,
        wall_stale_after: float | None = None,
    ) -> dict:
        wall_now = time.monotonic()
        now = wall_now if now_s is None else float(now_s)
        age = None if self.last == 0.0 else max(now - self.last, 0.0)
        wall_age = (
            None if self.last_wall == 0.0 else max(wall_now - self.last_wall, 0.0)
        )
        wall_healthy = (
            wall_stale_after is None
            or (
                wall_age is not None
                and wall_age <= max(float(wall_stale_after), 0.0)
            )
        )
        return {
            "healthy": age is not None and age <= stale_after and wall_healthy,
            "age_s": None if age is None else round(age, 3),
            "wall_age_s": None if wall_age is None else round(wall_age, 3),
            "rate_hz": round(self.filtered_rate, 1),
            "frames": self.count,
        }


class UavPerception(Node):
    """Publish one bounded body-frame avoidance vector and rich diagnostics."""

    def __init__(self) -> None:
        super().__init__("uav_perception")
        self.declare_parameter("scan_topic", "/uav/lidar3d/scan")
        self.declare_parameter("pointcloud_topic", "/uav/lidar3d/points")
        self.declare_parameter("depth_topic", "/uav/stereo/depth/depth_image")
        self.declare_parameter("tof_topic", "/uav/downward_tof/scan")
        self.declare_parameter("influence_distance_m", 3.0)
        self.declare_parameter("hard_stop_distance_m", 0.85)
        self.declare_parameter("sensor_stale_after_s", 0.7)
        self.declare_parameter("sensor_wall_stale_after_s", 1.5)
        self.declare_parameter("freshness_uses_ros_time", False)
        self.declare_parameter("point_stride", 4)
        self.declare_parameter("depth_stride", 8)
        self.declare_parameter("self_filter_radius_m", 0.36)

        self.influence = float(self.get_parameter("influence_distance_m").value)
        self.hard_stop_distance = float(
            self.get_parameter("hard_stop_distance_m").value
        )
        self.stale_after = float(self.get_parameter("sensor_stale_after_s").value)
        self.wall_stale_after = max(
            0.1, float(self.get_parameter("sensor_wall_stale_after_s").value)
        )
        self.freshness_uses_ros_time = bool(
            self.get_parameter("freshness_uses_ros_time").value
        )
        self.point_stride = max(int(self.get_parameter("point_stride").value), 1)
        self.depth_stride = max(int(self.get_parameter("depth_stride").value), 1)
        self.self_filter_radius = max(
            float(self.get_parameter("self_filter_radius_m").value), 0.05
        )

        names = [
            "lidar3d_scan",
            "lidar3d",
            "stereo_depth",
            "stereo_left",
            "stereo_right",
            "downward_camera",
            "downward_tof",
        ]
        self.rates = {name: RateTracker() for name in names}
        self.lidar2d_summary = self._empty_summary()
        self.lidar3d_summary = self._empty_summary()
        self.depth_summary = self._empty_summary()
        self.downward_range = math.inf
        self.tof_summary = self._empty_summary()
        self.last_summary = self._empty_summary()

        self.vector_publisher = self.create_publisher(
            Vector3Stamped, "/uav/perception/avoidance_vector", 10
        )
        self.status_publisher = self.create_publisher(
            String, "/uav/perception/status", 10
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("pointcloud_topic").value),
            self.on_pointcloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self.on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("tof_topic").value),
            self.on_tof,
            qos_profile_sensor_data,
        )
        for topic, key in (
            ("/uav/stereo/left/image_raw", "stereo_left"),
            ("/uav/stereo/right/image_raw", "stereo_right"),
            ("/vision/image_raw", "downward_camera"),
        ):
            self.create_subscription(
                Image,
                topic,
                lambda _message, sensor=key: self._mark_sensor(sensor),
                qos_profile_sensor_data,
            )
        self.publish_timer = create_steady_timer(self, 0.1, self.publish_fusion)

    @staticmethod
    def _empty_summary() -> ObstacleSummary:
        return ObstacleSummary(
            math.inf,
            {name: math.inf for name in DIRECTIONS},
            (0.0, 0.0, 0.0),
            0,
        )

    def _freshness_now(self) -> float:
        if self.freshness_uses_ros_time:
            return self.get_clock().now().nanoseconds / 1_000_000_000.0
        return time.monotonic()

    def _mark_sensor(self, sensor: str) -> None:
        self.rates[sensor].mark(self._freshness_now())

    def on_scan(self, message: LaserScan) -> None:
        self._mark_sensor("lidar3d_scan")
        self.lidar2d_summary = summarize_points(
            scan_to_points(message.ranges, message.angle_min, message.angle_increment),
            self.influence,
            0.7,
            self.self_filter_radius,
        )

    def _pointcloud_xyz(self, message: PointCloud2) -> Iterable[Tuple[float, float, float]]:
        offsets = {field.name: int(field.offset) for field in message.fields}
        if not all(name in offsets for name in ("x", "y", "z")):
            return
        endian = ">" if message.is_bigendian else "<"
        unpack = struct.Struct(endian + "fff").unpack_from
        contiguous = offsets["y"] == offsets["x"] + 4 and offsets["z"] == offsets["x"] + 8
        data = memoryview(message.data)
        total = int(message.width) * max(int(message.height), 1)
        for index in range(0, total, self.point_stride):
            offset = index * int(message.point_step)
            try:
                if contiguous:
                    yield unpack(data, offset + offsets["x"])
                else:
                    x = struct.unpack_from(endian + "f", data, offset + offsets["x"])[0]
                    y = struct.unpack_from(endian + "f", data, offset + offsets["y"])[0]
                    z = struct.unpack_from(endian + "f", data, offset + offsets["z"])[0]
                    yield (x, y, z)
            except (struct.error, ValueError):
                break

    def on_pointcloud(self, message: PointCloud2) -> None:
        self._mark_sensor("lidar3d")
        self.lidar3d_summary = summarize_points(
            self._pointcloud_xyz(message),
            self.influence,
            1.0,
            self.self_filter_radius,
        )

    def _depth_values(self, message: Image) -> Iterable[Tuple[float, int, int]]:
        encoding = message.encoding.upper()
        if encoding not in ("32FC1", "16UC1", "MONO16"):
            return
        endian = ">" if message.is_bigendian else "<"
        is_float = encoding == "32FC1"
        size = 4 if is_float else 2
        fmt = endian + ("f" if is_float else "H")
        data = memoryview(message.data)
        for row in range(0, int(message.height), self.depth_stride):
            row_offset = row * int(message.step)
            for column in range(0, int(message.width), self.depth_stride):
                try:
                    value = struct.unpack_from(fmt, data, row_offset + column * size)[0]
                except (struct.error, ValueError):
                    return
                distance = float(value) if is_float else float(value) / 1000.0
                if math.isfinite(distance) and 0.10 < distance < 30.0:
                    yield (distance, column, row)

    def on_depth(self, message: Image) -> None:
        self._mark_sensor("stereo_depth")
        width = max(int(message.width), 1)
        height = max(int(message.height), 1)
        points = []
        # Convert optical image coordinates into a coarse FLU ray. Exact
        # calibration remains in CameraInfo; this approximation is only for
        # short-horizon collision sectors.
        for distance, column, row in self._depth_values(message):
            horizontal = (0.5 - column / width) * 1.22
            vertical = (0.5 - row / height) * 0.72
            x = distance
            y = distance * math.tan(horizontal)
            z = distance * math.tan(vertical)
            points.append((x, y, z))
        self.depth_summary = summarize_points(points, self.influence, 0.65)

    def on_tof(self, message: LaserScan) -> None:
        valid = [
            float(value)
            for value in message.ranges
            if math.isfinite(value)
            and float(message.range_min) <= float(value) <= float(message.range_max)
        ]
        if not valid:
            return
        valid.sort()
        self.downward_range = valid[len(valid) // 2]
        self.tof_summary = summarize_points(
            ((0.0, 0.0, -self.downward_range),), self.influence, 0.9
        )
        self._mark_sensor("downward_tof")

    def _sensor_report(self, sensor: str, now_s: float) -> dict:
        return self.rates[sensor].report(
            self.stale_after,
            now_s=now_s,
            wall_stale_after=self.wall_stale_after,
        )

    def _fresh_summary(
        self, sensor: str, summary: ObstacleSummary, now_s: float
    ) -> ObstacleSummary:
        if self._sensor_report(sensor, now_s)["healthy"]:
            return summary
        return self._empty_summary()

    def publish_fusion(self) -> None:
        freshness_now = self._freshness_now()
        summary = combine_summaries(
            (
                self._fresh_summary("lidar3d_scan", self.lidar2d_summary, freshness_now),
                self._fresh_summary("lidar3d", self.lidar3d_summary, freshness_now),
                self._fresh_summary("stereo_depth", self.depth_summary, freshness_now),
                self._fresh_summary("downward_tof", self.tof_summary, freshness_now),
            ),
            1.0,
        )
        self.last_summary = summary

        vector = Vector3Stamped()
        vector.header.stamp = self.get_clock().now().to_msg()
        vector.header.frame_id = "uav_base_link"
        vector.vector.x, vector.vector.y, vector.vector.z = summary.repulsion
        self.vector_publisher.publish(vector)

        sensors = {
            name: self._sensor_report(name, freshness_now) for name in self.rates
        }
        primary_healthy = sensors["lidar3d_scan"]["healthy"] and (
            sensors["lidar3d"]["healthy"] or sensors["stereo_depth"]["healthy"]
        ) and sensors["downward_camera"]["healthy"] and sensors["downward_tof"]["healthy"]
        degraded = [name for name, state in sensors.items() if not state["healthy"]]
        message = String()
        message.data = json.dumps(
            {
                "healthy": primary_healthy,
                "freshness_clock": (
                    "ros_time_with_steady_dead_link"
                    if self.freshness_uses_ros_time
                    else "steady_time"
                ),
                "sensor_stale_after_s": self.stale_after,
                "sensor_wall_stale_after_s": self.wall_stale_after,
                "degraded_sensors": degraded,
                "minimum_obstacle_m": None
                if not math.isfinite(summary.minimum_m)
                else round(summary.minimum_m, 3),
                "hard_stop": math.isfinite(summary.minimum_m)
                and summary.minimum_m <= self.hard_stop_distance,
                "hard_stop_distance_m": self.hard_stop_distance,
                "repulsion_body": [round(value, 4) for value in summary.repulsion],
                "sectors_m": {
                    name: None if not math.isfinite(value) else round(value, 3)
                    for name, value in summary.sectors.items()
                },
                "point_count": summary.point_count,
                "source_minimums_m": {
                    "lidar3d_scan": None
                    if not math.isfinite(self.lidar2d_summary.minimum_m)
                    else round(self.lidar2d_summary.minimum_m, 3),
                    "lidar3d": None
                    if not math.isfinite(self.lidar3d_summary.minimum_m)
                    else round(self.lidar3d_summary.minimum_m, 3),
                    "stereo_depth": None
                    if not math.isfinite(self.depth_summary.minimum_m)
                    else round(self.depth_summary.minimum_m, 3),
                    "downward_tof": None
                    if not math.isfinite(self.downward_range)
                    else round(self.downward_range, 3),
                },
                "downward_range_m": None
                if not math.isfinite(self.downward_range)
                else round(self.downward_range, 3),
                "sensors": sensors,
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UavPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
