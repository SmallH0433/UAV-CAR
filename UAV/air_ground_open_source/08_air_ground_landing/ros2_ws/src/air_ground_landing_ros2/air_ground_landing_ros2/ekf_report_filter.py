"""Isolate ArduPilot EKF reports from the high-rate MAVROS raw stream."""

from __future__ import annotations

import rclpy
from mavros_msgs.msg import Mavlink
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from air_ground_landing.mavlink_ekf import EKF_STATUS_REPORT_ID, report_health


class EkfReportFilter(Node):
    def __init__(self) -> None:
        super().__init__("ekf_report_filter")
        self.declare_parameter("source_topic", "/uas1/mavlink_source")
        self.declare_parameter("report_topic", "/landing/ekf_report")
        self.publisher = self.create_publisher(
            Mavlink, str(self.get_parameter("report_topic").value), qos_profile_sensor_data
        )
        self.create_subscription(
            Mavlink,
            str(self.get_parameter("source_topic").value),
            self._source,
            qos_profile_sensor_data,
        )

    def _source(self, message: Mavlink) -> None:
        if message.msgid != EKF_STATUS_REPORT_ID:
            return
        health = report_health(
            framing_status=int(message.framing_status),
            system_id=int(message.sysid),
            component_id=int(message.compid),
            message_id=int(message.msgid),
            length=int(message.len),
            payload64=message.payload64,
        )
        if health is not None:
            # Forward unhealthy reports too; the executor must reject them.
            self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EkfReportFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
