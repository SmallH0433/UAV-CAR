"""Bound Gazebo clock fan-out without changing simulation time semantics."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

from .runtime_timing import create_steady_timer


@dataclass
class ClockThrottle:
    """Decide when a simulation clock sample should be forwarded."""

    period_ns: int
    keepalive_s: float
    last_published_sim_ns: Optional[int] = None
    last_published_wall_s: Optional[float] = None

    def accept(self, sim_ns: int, wall_s: float) -> bool:
        current = int(sim_ns)
        if (
            self.last_published_sim_ns is None
            or current < self.last_published_sim_ns
            or current - self.last_published_sim_ns >= self.period_ns
        ):
            self.mark_published(current, wall_s)
            return True
        return False

    def keepalive_due(self, wall_s: float) -> bool:
        return bool(
            self.last_published_sim_ns is not None
            and self.last_published_wall_s is not None
            and float(wall_s) - self.last_published_wall_s >= self.keepalive_s
        )

    def mark_published(self, sim_ns: int, wall_s: float) -> None:
        self.last_published_sim_ns = int(sim_ns)
        self.last_published_wall_s = float(wall_s)


class SimulationClockRelay(Node):
    """Forward Gazebo time at a bounded rate to all ROS-time consumers.

    Gazebo emits a clock sample at every 1 ms physics step. Sending every
    sample to every Python, Nav2 and TF process wastes substantial CPU without
    improving the 20--50 Hz control and sensor loops. The relay keeps the exact
    Gazebo timestamp while bounding DDS fan-out. Safety and command watchdogs
    remain on independent steady clocks.
    """

    def __init__(self) -> None:
        super().__init__("simulation_clock_relay")
        self.declare_parameter("input_topic", "/clock_raw")
        self.declare_parameter("output_topic", "/clock")
        self.declare_parameter("max_rate_hz", 100.0)
        self.declare_parameter("paused_keepalive_s", 0.5)

        rate_hz = float(self.get_parameter("max_rate_hz").value)
        keepalive_s = float(self.get_parameter("paused_keepalive_s").value)
        if rate_hz <= 0.0 or keepalive_s <= 0.0:
            raise ValueError("clock relay rates must be positive")

        self.throttle = ClockThrottle(
            period_ns=max(1, round(1_000_000_000 / rate_hz)),
            keepalive_s=keepalive_s,
        )
        self.latest: Optional[Clock] = None

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            Clock, str(self.get_parameter("output_topic").value), output_qos
        )
        self.subscription = self.create_subscription(
            Clock,
            str(self.get_parameter("input_topic").value),
            self.on_clock,
            input_qos,
        )
        self.keepalive_timer = create_steady_timer(
            self, min(keepalive_s / 2.0, 0.25), self.publish_keepalive
        )
        self.get_logger().info(
            f"Simulation clock relay limited to {rate_hz:.1f} Hz; "
            f"paused keepalive={keepalive_s:.2f}s"
        )

    @staticmethod
    def _nanoseconds(message: Clock) -> int:
        return int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)

    def on_clock(self, message: Clock) -> None:
        self.latest = message
        now = time.monotonic()
        sim_ns = self._nanoseconds(message)
        if self.throttle.accept(sim_ns, now):
            self.publisher.publish(message)

    def publish_keepalive(self) -> None:
        now = time.monotonic()
        if self.latest is None or not self.throttle.keepalive_due(now):
            return
        self.publisher.publish(self.latest)
        self.throttle.mark_published(self._nanoseconds(self.latest), now)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimulationClockRelay()
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
