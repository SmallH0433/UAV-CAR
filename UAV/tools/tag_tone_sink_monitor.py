#!/usr/bin/env python3
"""Print the first legacy PLAY_TUNE frame observed on the MAVROS sink."""

import json
import time

import rclpy
from mavros_msgs.msg import Mavlink
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def main() -> None:
    rclpy.init()
    node = Node("tag_tone_sink_monitor")
    observed: dict[str, object] = {}

    def receive(message: Mavlink) -> None:
        if int(message.msgid) != 258 or observed:
            return
        observed.update(
            {
                "msgid": int(message.msgid),
                "magic": int(message.magic),
                "payload_length": int(message.len),
                "source_system": int(message.sysid),
                "source_component": int(message.compid),
                "checksum": int(message.checksum),
                "payload64": [int(value) for value in message.payload64],
            }
        )

    node.create_subscription(
        Mavlink,
        "/uas1/mavlink_sink",
        receive,
        qos_profile_sensor_data,
    )
    deadline = time.monotonic() + 20.0
    while not observed and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(json.dumps(observed, separators=(",", ":")), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    if not observed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
