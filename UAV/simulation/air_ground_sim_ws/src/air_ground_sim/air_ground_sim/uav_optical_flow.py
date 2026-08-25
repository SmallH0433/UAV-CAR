"""Estimate metric body-frame velocity from the downward OV9281 and ToF."""

from __future__ import annotations

import json
import math
import time

from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import TwistStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String


class UavOpticalFlow(Node):
    """Publish range-scaled optical-flow velocity without using ground truth."""

    def __init__(self) -> None:
        super().__init__("uav_optical_flow")
        self.declare_parameter("image_topic", "/vision/image_raw")
        self.declare_parameter("camera_info_topic", "/vision/camera_info")
        self.declare_parameter("tof_topic", "/uav/downward_tof/scan")
        self.declare_parameter("velocity_topic", "/uav/optical_flow/velocity")
        self.declare_parameter("minimum_features", 24)
        self.declare_parameter("maximum_features", 180)
        self.declare_parameter("quality_level", 0.015)
        self.declare_parameter("minimum_feature_distance_px", 8.0)
        self.declare_parameter("tof_timeout_s", 0.25)
        self.declare_parameter("forward_sign", -1.0)
        self.declare_parameter("left_sign", -1.0)

        self.minimum_features = max(
            8, int(self.get_parameter("minimum_features").value)
        )
        self.maximum_features = max(
            self.minimum_features,
            int(self.get_parameter("maximum_features").value),
        )
        self.quality_level = max(
            0.001, float(self.get_parameter("quality_level").value)
        )
        self.minimum_feature_distance = max(
            2.0, float(self.get_parameter("minimum_feature_distance_px").value)
        )
        self.tof_timeout = max(0.05, float(self.get_parameter("tof_timeout_s").value))
        self.forward_sign = float(self.get_parameter("forward_sign").value)
        self.left_sign = float(self.get_parameter("left_sign").value)

        self.bridge = CvBridge()
        self.previous_image = None
        self.previous_points = None
        self.previous_stamp = None
        self.focal_x = None
        self.focal_y = None
        self.downward_range = None
        self.last_tof_wall = 0.0

        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            str(self.get_parameter("velocity_topic").value),
            qos_profile_sensor_data,
        )
        self.status_publisher = self.create_publisher(
            String, "/uav/optical_flow/status", 10
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("tof_topic").value),
            self.on_tof,
            qos_profile_sensor_data,
        )

    def on_camera_info(self, message: CameraInfo) -> None:
        if len(message.k) >= 6 and message.k[0] > 0.0 and message.k[4] > 0.0:
            self.focal_x = float(message.k[0])
            self.focal_y = float(message.k[4])

    def on_tof(self, message: LaserScan) -> None:
        values = sorted(
            float(value)
            for value in message.ranges
            if math.isfinite(value)
            and float(message.range_min) <= float(value) <= float(message.range_max)
        )
        if values:
            self.downward_range = values[len(values) // 2]
            self.last_tof_wall = time.monotonic()

    @staticmethod
    def _stamp_seconds(message: Image) -> float:
        return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9

    def _new_features(self, image: np.ndarray):
        return cv2.goodFeaturesToTrack(
            image,
            maxCorners=self.maximum_features,
            qualityLevel=self.quality_level,
            minDistance=self.minimum_feature_distance,
            blockSize=7,
        )

    def _publish_status(self, healthy: bool, reason: str, tracks: int = 0) -> None:
        message = String()
        message.data = json.dumps(
            {
                "healthy": healthy,
                "reason": reason,
                "tracked_features": tracks,
                "range_m": self.downward_range,
                "model": "ov9281_lk_flow_with_downward_tof_scale",
            },
            sort_keys=True,
        )
        self.status_publisher.publish(message)

    def on_image(self, message: Image) -> None:
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
        stamp = self._stamp_seconds(message)
        if self.focal_x is None or self.focal_y is None:
            # The configured 130-degree horizontal lens is the startup fallback
            # until CameraInfo arrives from the Gazebo bridge.
            self.focal_x = image.shape[1] / (2.0 * math.tan(math.radians(65.0)))
            self.focal_y = self.focal_x

        if self.previous_image is None or self.previous_points is None:
            self.previous_image = image
            self.previous_points = self._new_features(image)
            self.previous_stamp = stamp
            self._publish_status(False, "initializing")
            return

        dt = stamp - float(self.previous_stamp)
        tof_fresh = (
            self.downward_range is not None
            and time.monotonic() - self.last_tof_wall <= self.tof_timeout
        )
        next_points, status, _error = cv2.calcOpticalFlowPyrLK(
            self.previous_image,
            image,
            self.previous_points,
            None,
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        good = status.reshape(-1).astype(bool) if status is not None else np.zeros(0, dtype=bool)
        previous_good = self.previous_points.reshape(-1, 2)[good]
        next_good = next_points.reshape(-1, 2)[good] if next_points is not None else np.empty((0, 2))
        track_count = int(len(next_good))

        if dt > 0.0 and tof_fresh and track_count >= self.minimum_features:
            displacement = np.median(next_good - previous_good, axis=0)
            velocity = TwistStamped()
            velocity.header = message.header
            velocity.header.frame_id = "uav_base_link"
            velocity.twist.linear.x = self.forward_sign * float(displacement[1]) * float(self.downward_range) / (float(self.focal_y) * dt)
            velocity.twist.linear.y = self.left_sign * float(displacement[0]) * float(self.downward_range) / (float(self.focal_x) * dt)
            self.velocity_publisher.publish(velocity)
            self._publish_status(True, "tracking", track_count)
        else:
            reason = "tof_stale" if not tof_fresh else "insufficient_features"
            self._publish_status(False, reason, track_count)

        self.previous_image = image
        self.previous_points = (
            next_good.reshape(-1, 1, 2).astype(np.float32)
            if track_count >= self.minimum_features
            else self._new_features(image)
        )
        self.previous_stamp = stamp


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UavOpticalFlow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
