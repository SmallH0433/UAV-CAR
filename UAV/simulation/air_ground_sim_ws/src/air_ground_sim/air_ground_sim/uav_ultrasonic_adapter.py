"""Convert shared Gazebo geometry into realistic ultrasonic Range topics."""

from collections import deque
import json
import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

from .perception_math import DIRECTIONS, bounded_range, directional_cone_ranges
from .runtime_timing import create_steady_timer


class UavUltrasonicAdapter(Node):
    """Model range saturation, noise, delay and dropout at the ROS boundary."""

    def __init__(self) -> None:
        super().__init__("uav_ultrasonic_adapter")
        self.declare_parameter("pointcloud_topic", "/uav/lidar3d/points")
        self.declare_parameter("output_prefix", "/uav/range")
        self.declare_parameter("minimum_range_m", 0.20)
        self.declare_parameter("maximum_range_m", 6.0)
        self.declare_parameter("field_of_view_rad", 0.40)
        self.declare_parameter("noise_stddev_m", 0.015)
        self.declare_parameter("latency_ms", 18.0)
        self.declare_parameter("dropout_probability", 0.01)
        self.declare_parameter("random_seed", 41)
        self.declare_parameter("status_stale_after_s", 0.25)

        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.output_prefix = str(self.get_parameter("output_prefix").value).rstrip("/")
        self.minimum = float(self.get_parameter("minimum_range_m").value)
        self.maximum = float(self.get_parameter("maximum_range_m").value)
        self.field_of_view = float(self.get_parameter("field_of_view_rad").value)
        self.noise = max(float(self.get_parameter("noise_stddev_m").value), 0.0)
        self.latency = max(float(self.get_parameter("latency_ms").value), 0.0) / 1000.0
        self.dropout = min(
            max(float(self.get_parameter("dropout_probability").value), 0.0), 1.0
        )
        self.random = random.Random(int(self.get_parameter("random_seed").value))
        self.status_stale_after = max(
            float(self.get_parameter("status_stale_after_s").value), 0.05
        )

        self.range_publishers = {}
        self.pending = deque(maxlen=256)
        self.last_input = {name: 0.0 for name in DIRECTIONS}
        self.last_output = {name: 0.0 for name in DIRECTIONS}
        self.last_range = {name: math.inf for name in DIRECTIONS}
        self.frames = {name: 0 for name in DIRECTIONS}
        self.dropped = {name: 0 for name in DIRECTIONS}

        for name in DIRECTIONS:
            self.range_publishers[name] = self.create_publisher(
                Range, f"{self.output_prefix}/{name}", qos_profile_sensor_data
            )
        self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.on_pointcloud,
            qos_profile_sensor_data,
        )
        self.status_publisher = self.create_publisher(
            String, "/uav/ultrasonic/status", 10
        )
        self.flush_timer = create_steady_timer(self, 0.005, self.flush_pending)
        self.status_timer = create_steady_timer(self, 0.5, self.publish_status)

    def on_pointcloud(self, cloud: PointCloud2) -> None:
        points = point_cloud2.read_points(
            cloud, field_names=("x", "y", "z"), skip_nans=True
        )
        cone_ranges = directional_cone_ranges(
            points,
            self.field_of_view,
            self.minimum,
            self.maximum,
        )
        now = time.monotonic()
        for direction in DIRECTIONS:
            self.last_input[direction] = now
            self.frames[direction] += 1
            if self.random.random() < self.dropout:
                self.dropped[direction] += 1
                continue

            distance = float(cone_ranges[direction])
            if self.noise > 0.0 and math.isfinite(distance):
                distance += self.random.gauss(0.0, self.noise)
            distance = bounded_range(distance, self.minimum, self.maximum)

            output = Range()
            output.header.stamp = cloud.header.stamp
            output.header.frame_id = f"uav_ultrasonic_{direction}_frame"
            output.radiation_type = Range.ULTRASOUND
            output.field_of_view = self.field_of_view
            output.min_range = self.minimum
            output.max_range = self.maximum
            output.range = distance
            self.pending.append((now + self.latency, direction, output))

    def flush_pending(self) -> None:
        now = time.monotonic()
        while self.pending and self.pending[0][0] <= now:
            _, direction, message = self.pending.popleft()
            self.range_publishers[direction].publish(message)
            self.last_output[direction] = now
            self.last_range[direction] = float(message.range)

    def publish_status(self) -> None:
        now = time.monotonic()
        sensors = {}
        for name in DIRECTIONS:
            age = None if self.last_output[name] == 0.0 else now - self.last_output[name]
            sensors[name] = {
                "healthy": age is not None and age <= self.status_stale_after,
                "age_s": None if age is None else round(age, 3),
                "range_m": None
                if not math.isfinite(self.last_range[name])
                else round(self.last_range[name], 3),
                "frames": self.frames[name],
                "dropped": self.dropped[name],
            }
        message = String()
        message.data = json.dumps(
            {
                "model": "shared_3d_geometry_with_independent_ultrasonic_transport",
                "geometry_source": self.pointcloud_topic,
                "latency_ms": round(self.latency * 1000.0, 1),
                "noise_stddev_m": self.noise,
                "sensors": sensors,
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UavUltrasonicAdapter()
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
