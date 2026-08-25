#!/usr/bin/env python3
"""Publish only a simulated accepted-tag status for audible-tone verification."""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main() -> None:
    rclpy.init()
    node = Node("tag_tone_smoke")
    publisher = node.create_publisher(String, "/landing/landing_target/status", 10)
    message = String()
    message.data = json.dumps(
        {
            "node": "TAG_TONE_SMOKE_ONLY",
            "accepted_this_poll": True,
            "stream_healthy": True,
            "mavlink_transmitted": False,
        },
        separators=(",", ":"),
    )
    discovery_deadline = time.monotonic() + 2.0
    while time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    end_s = time.monotonic() + 5.2
    while time.monotonic() < end_s:
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
