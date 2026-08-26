"""Minimal ROS 2 owner coordinator for IBVS follow and AC_PrecLand handover."""

from __future__ import annotations

import json
import time
from typing import Optional

import rclpy
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node
from std_msgs.msg import Bool, String

from air_ground_landing.simple_coordination import (
    SimpleCoordinationConfig,
    select_simple_owner,
)


class SimpleLandingCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("simple_landing_coordinator")
        defaults = {
            "ibvs_candidate_topic": "/landing/ibvs/candidate",
            "landing_request_topic": "/landing/descent_request",
            "landing_target_status_topic": "/landing/landing_target/status",
            "mavros_state_topic": "/mavros/state",
            "control_owner_topic": "/landing/control_owner",
            "status_topic": "/landing/simple_coordinator/status",
            "ibvs_timeout_s": 0.25,
            "landing_target_timeout_s": 0.35,
            "require_landing_target_output": False,
            "output_rate_hz": 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config = SimpleCoordinationConfig(
            ibvs_timeout_s=float(self.get_parameter("ibvs_timeout_s").value),
            landing_target_timeout_s=float(
                self.get_parameter("landing_target_timeout_s").value
            ),
        )
        self.config.validate()
        rate_hz = float(self.get_parameter("output_rate_hz").value)
        if not 1.0 <= rate_hz <= 50.0:
            raise ValueError("output_rate_hz must be in [1, 50]")

        self.connected = False
        self.descent_requested = False
        self.ibvs_received_s: Optional[float] = None
        self.landing_target_received_s: Optional[float] = None
        self.landing_target_healthy = False
        self.landing_target_output_enabled = False
        self.require_landing_target_output = bool(
            self.get_parameter("require_landing_target_output").value
        )

        self.owner_publisher = self.create_publisher(
            String, str(self.get_parameter("control_owner_topic").value), 10
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.create_subscription(
            PositionTarget,
            str(self.get_parameter("ibvs_candidate_topic").value),
            self._ibvs,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("landing_request_topic").value),
            self._landing_request,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("landing_target_status_topic").value),
            self._landing_target_status,
            10,
        )
        self.create_subscription(
            State,
            str(self.get_parameter("mavros_state_topic").value),
            self._state,
            10,
        )
        self.create_timer(1.0 / rate_hz, self._tick)

    @staticmethod
    def _now_s() -> float:
        return time.monotonic()

    def _ibvs(self, _message: PositionTarget) -> None:
        self.ibvs_received_s = self._now_s()

    def _landing_request(self, message: Bool) -> None:
        self.descent_requested = bool(message.data)

    def _landing_target_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            self.landing_target_healthy = False
            self.landing_target_output_enabled = False
            return
        self.landing_target_output_enabled = bool(
            payload.get("output_enabled", False)
        )
        self.landing_target_healthy = bool(
            payload.get("stream_healthy", False)
            and (
                not self.require_landing_target_output
                or self.landing_target_output_enabled
            )
        )
        if self.landing_target_healthy:
            self.landing_target_received_s = self._now_s()

    def _state(self, message: State) -> None:
        self.connected = bool(message.connected)

    def _tick(self) -> None:
        now_s = self._now_s()
        ibvs_age_s = (
            None if self.ibvs_received_s is None else now_s - self.ibvs_received_s
        )
        target_age_s = (
            None
            if self.landing_target_received_s is None
            else now_s - self.landing_target_received_s
        )
        decision = select_simple_owner(
            connected=self.connected,
            descent_requested=self.descent_requested,
            ibvs_age_s=ibvs_age_s,
            landing_target_age_s=target_age_s,
            landing_target_healthy=self.landing_target_healthy,
            config=self.config,
        )
        owner_message = String()
        owner_message.data = decision.owner.value
        self.owner_publisher.publish(owner_message)
        status_message = String()
        status_message.data = json.dumps(
            {
                "node": "SIMPLE_LANDING_COORDINATOR_ROS2",
                "owner": decision.owner.value,
                "reason": decision.reason,
                "connected": self.connected,
                "descent_requested": self.descent_requested,
                "ibvs_age_s": ibvs_age_s,
                "landing_target_age_s": target_age_s,
                "landing_target_healthy": self.landing_target_healthy,
                "landing_target_output_required": self.require_landing_target_output,
                "landing_target_output_enabled": self.landing_target_output_enabled,
            },
            separators=(",", ":"),
        )
        self.status_publisher.publish(status_message)


def main() -> None:
    rclpy.init()
    node = SimpleLandingCoordinator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
