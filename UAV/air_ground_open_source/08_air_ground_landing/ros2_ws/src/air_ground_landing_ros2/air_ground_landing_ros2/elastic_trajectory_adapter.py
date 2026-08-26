"""Adapt a ROS 2 Elastic-style trajectory to MAVROS setpoint candidates."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import MultiDOFJointTrajectory


@dataclass(frozen=True)
class Sample:
    time_s: float
    position: tuple[float, float, float]
    velocity: Optional[tuple[float, float, float]]
    acceleration: Optional[tuple[float, float, float]]
    yaw: float
    yaw_rate: Optional[float]


def _duration_s(value) -> float:
    return float(value.sec) + float(value.nanosec) * 1.0e-9


def _yaw(rotation) -> float:
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _lerp(a: float, b: float, ratio: float) -> float:
    return a + (b - a) * ratio


def _lerp3(a, b, ratio: float):
    return tuple(_lerp(float(a[index]), float(b[index]), ratio) for index in range(3))


def _lerp_optional(a, b, ratio: float):
    if a is None or b is None:
        return a if ratio < 0.5 else b
    return _lerp3(a, b, ratio)


def _lerp_angle(a: float, b: float, ratio: float) -> float:
    delta = (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return a + delta * ratio


class ElasticTrajectoryAdapter(Node):
    """Sample standard ROS 2 trajectories; never publish directly to MAVROS."""

    def __init__(self) -> None:
        super().__init__("elastic_trajectory_adapter")
        self.declare_parameter("input_topic", "/elastic_tracker/trajectory")
        self.declare_parameter("candidate_topic", "/landing/elastic/candidate")
        self.declare_parameter("status_topic", "/landing/elastic/status")
        self.declare_parameter("output_rate_hz", 20.0)
        self.declare_parameter("maximum_trajectory_age_s", 0.5)
        input_topic = self.get_parameter("input_topic").value
        candidate_topic = self.get_parameter("candidate_topic").value
        status_topic = self.get_parameter("status_topic").value
        rate_hz = float(self.get_parameter("output_rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("output_rate_hz must be positive")
        self.maximum_trajectory_age_s = float(
            self.get_parameter("maximum_trajectory_age_s").value
        )
        self.samples: tuple[Sample, ...] = ()
        self.start_time_s = 0.0
        self.trajectory_sequence = 0
        self.publisher = self.create_publisher(PositionTarget, candidate_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.create_subscription(MultiDOFJointTrajectory, input_topic, self._trajectory, 10)
        self.create_timer(1.0 / rate_hz, self._tick)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _trajectory(self, message: MultiDOFJointTrajectory) -> None:
        parsed: list[Sample] = []
        for point in message.points:
            if not point.transforms:
                continue
            transform = point.transforms[0]
            velocity = point.velocities[0] if point.velocities else None
            acceleration = point.accelerations[0] if point.accelerations else None
            parsed.append(
                Sample(
                    time_s=_duration_s(point.time_from_start),
                    position=(
                        float(transform.translation.x),
                        float(transform.translation.y),
                        float(transform.translation.z),
                    ),
                    velocity=None
                    if velocity is None
                    else (
                        float(velocity.linear.x),
                        float(velocity.linear.y),
                        float(velocity.linear.z),
                    ),
                    acceleration=None
                    if acceleration is None
                    else (
                        float(acceleration.linear.x),
                        float(acceleration.linear.y),
                        float(acceleration.linear.z),
                    ),
                    yaw=_yaw(transform.rotation),
                    yaw_rate=None if velocity is None else float(velocity.angular.z),
                )
            )
        if not parsed or any(
            parsed[index].time_s >= parsed[index + 1].time_s
            for index in range(len(parsed) - 1)
        ):
            self.samples = ()
            self._publish_status(False, "EMPTY_OR_NON_MONOTONIC_TRAJECTORY")
            return
        self.samples = tuple(parsed)
        header_time = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1.0e-9
        now_s = self._now_s()
        self.start_time_s = header_time if 0.0 < now_s - header_time < self.maximum_trajectory_age_s else now_s
        self.trajectory_sequence += 1
        self._publish_status(True, "TRAJECTORY_ACCEPTED")

    def _sample(self, elapsed_s: float) -> Optional[Sample]:
        if not self.samples or elapsed_s < self.samples[0].time_s:
            return None
        if elapsed_s > self.samples[-1].time_s:
            return None
        for index in range(len(self.samples) - 1):
            left, right = self.samples[index], self.samples[index + 1]
            if left.time_s <= elapsed_s <= right.time_s:
                ratio = (elapsed_s - left.time_s) / (right.time_s - left.time_s)
                velocity = _lerp_optional(left.velocity, right.velocity, ratio)
                acceleration = _lerp_optional(left.acceleration, right.acceleration, ratio)
                yaw_rate = (
                    None
                    if left.yaw_rate is None or right.yaw_rate is None
                    else _lerp(left.yaw_rate, right.yaw_rate, ratio)
                )
                return Sample(
                    elapsed_s,
                    _lerp3(left.position, right.position, ratio),
                    velocity,
                    acceleration,
                    _lerp_angle(left.yaw, right.yaw, ratio),
                    yaw_rate,
                )
        return self.samples[-1]

    def _tick(self) -> None:
        sample = self._sample(self._now_s() - self.start_time_s)
        if sample is None:
            return
        target = PositionTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        # Values on this ROS topic remain ENU.  MAVROS converts them to the
        # declared MAV_FRAME_LOCAL_NED when the executor forwards the message.
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.position.x, target.position.y, target.position.z = sample.position
        target.yaw = float(sample.yaw)
        target.type_mask = PositionTarget.IGNORE_YAW_RATE
        if sample.velocity is None:
            target.type_mask |= (
                PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
            )
        else:
            target.velocity.x, target.velocity.y, target.velocity.z = sample.velocity
            if sample.yaw_rate is not None:
                target.type_mask &= ~PositionTarget.IGNORE_YAW_RATE
                target.yaw_rate = float(sample.yaw_rate)
        if sample.acceleration is None:
            target.type_mask |= (
                PositionTarget.IGNORE_AFX
                | PositionTarget.IGNORE_AFY
                | PositionTarget.IGNORE_AFZ
            )
        else:
            (
                target.acceleration_or_force.x,
                target.acceleration_or_force.y,
                target.acceleration_or_force.z,
            ) = sample.acceleration
        self.publisher.publish(target)

    def _publish_status(self, healthy: bool, reason: str) -> None:
        message = String()
        message.data = json.dumps(
            {
                "source": "ELASTIC_ROS2_ADAPTER",
                "healthy": healthy,
                "reason": reason,
                "trajectory_sequence": self.trajectory_sequence,
                "point_count": len(self.samples),
            },
            separators=(",", ":"),
        )
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ElasticTrajectoryAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
