"""Small health monitor for the common simulation / IMX296 image topic."""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .runtime_timing import create_steady_timer


class VisionInputMonitor(Node):
    """Publish camera rate and age without copying or modifying image data."""

    def __init__(self) -> None:
        super().__init__("vision_input_monitor")
        self.declare_parameter("image_topic", "/vision/image_raw")
        self.declare_parameter("stale_after_s", 1.0)
        self.declare_parameter("reliable_qos", False)
        topic = str(self.get_parameter("image_topic").value)
        self.stale_after = float(self.get_parameter("stale_after_s").value)
        image_qos = (
            10
            if bool(self.get_parameter("reliable_qos").value)
            else qos_profile_sensor_data
        )
        self.frames = 0
        self.previous_frames = 0
        self.last_frame_monotonic = 0.0
        self.width = 0
        self.height = 0
        self.subscription = self.create_subscription(
            Image, topic, self.on_image, image_qos
        )
        self.publisher = self.create_publisher(String, "/vision/status", 10)
        self.timer = create_steady_timer(self, 1.0, self.publish_status)
        self.get_logger().info(f"Monitoring visual input on {topic}")

    def on_image(self, message: Image) -> None:
        self.frames += 1
        self.last_frame_monotonic = time.monotonic()
        self.width = int(message.width)
        self.height = int(message.height)

    def publish_status(self) -> None:
        now = time.monotonic()
        age = None if self.last_frame_monotonic == 0.0 else now - self.last_frame_monotonic
        fps = self.frames - self.previous_frames
        self.previous_frames = self.frames
        payload = {
            "healthy": age is not None and age <= self.stale_after,
            "age_s": None if age is None else round(age, 3),
            "fps_last_second": fps,
            "frames_total": self.frames,
            "width": self.width,
            "height": self.height,
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionInputMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
