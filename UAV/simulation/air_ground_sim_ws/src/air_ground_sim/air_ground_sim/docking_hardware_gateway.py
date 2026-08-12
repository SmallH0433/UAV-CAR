"""Fail-closed ROS gateway for a redundant physical UAV docking latch."""

from __future__ import annotations

import json
import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import SetBool

from .docking_hardware_logic import (
    dual_channel_state,
    feedback_safe_for_enable,
    operation_command_decision,
    physical_attach_authorized,
    physical_release_authorized,
)
from .runtime_timing import create_steady_timer


def _json(message: String) -> dict:
    try:
        value = json.loads(message.data)
    except (json.JSONDecodeError, TypeError):
        return {}
    return value if isinstance(value, dict) else {}


class DockingHardwareGateway(Node):
    """Translate guarded mission requests into one physical lock command."""

    def __init__(self) -> None:
        super().__init__("docking_hardware_gateway")
        self.declare_parameter("command_enabled", False)
        self.declare_parameter("attach_topic", "/uav/dock/attach")
        self.declare_parameter("detach_topic", "/uav/dock/detach")
        self.declare_parameter("lock_command_topic", "/dock_hw/lock_command")
        self.declare_parameter("contact_a_topic", "/dock_hw/contact_a")
        self.declare_parameter("contact_b_topic", "/dock_hw/contact_b")
        self.declare_parameter("locked_a_topic", "/dock_hw/locked_a")
        self.declare_parameter("locked_b_topic", "/dock_hw/locked_b")
        self.declare_parameter("emergency_stop_topic", "/system/emergency_stop")
        self.declare_parameter("feedback_timeout_s", 0.30)
        self.declare_parameter("state_timeout_s", 0.75)
        self.declare_parameter("operation_timeout_s", 2.0)
        self.declare_parameter("startup_grace_s", 8.0)
        self.declare_parameter("stationary_speed_limit_mps", 0.03)
        self.declare_parameter("moving_speed_limit_mps", 0.15)
        self.declare_parameter("moving_yaw_rate_limit_rps", 0.20)
        self.declare_parameter("moving_capture_max_altitude_m", 0.50)

        self.command_enabled = bool(self.get_parameter("command_enabled").value)
        self.feedback_timeout = max(
            0.05, float(self.get_parameter("feedback_timeout_s").value)
        )
        self.state_timeout = max(
            0.1, float(self.get_parameter("state_timeout_s").value)
        )
        self.operation_timeout = max(
            0.1, float(self.get_parameter("operation_timeout_s").value)
        )
        self.startup_grace = max(
            0.0, float(self.get_parameter("startup_grace_s").value)
        )
        self.stationary_speed_limit = max(
            0.0, float(self.get_parameter("stationary_speed_limit_mps").value)
        )
        self.moving_speed_limit = max(
            0.0, float(self.get_parameter("moving_speed_limit_mps").value)
        )
        self.moving_yaw_rate_limit = max(
            0.0, float(self.get_parameter("moving_yaw_rate_limit_rps").value)
        )
        self.moving_capture_max_altitude = max(
            0.0,
            float(self.get_parameter("moving_capture_max_altitude_m").value),
        )

        self.telemetry: dict = {}
        self.mission: dict = {}
        self.telemetry_time = 0.0
        self.mission_time = 0.0
        self.ugv_speed = 0.0
        self.ugv_yaw_rate = 0.0
        self.ugv_time = 0.0
        # Unknown safety state is fail-closed until the supervisor publishes.
        self.emergency_stop = True
        self.emergency_stop_time = 0.0
        self.feedback = {
            "contact_a": False,
            "contact_b": False,
            "locked_a": False,
            "locked_b": False,
        }
        self.feedback_times = {key: 0.0 for key in self.feedback}
        self.commanded_locked: bool | None = None
        self.operation_started = 0.0
        self.last_request = "none"
        self.last_result = "waiting_for_feedback"
        self.boot_time = time.monotonic()

        self.lock_publisher = self.create_publisher(
            Bool, str(self.get_parameter("lock_command_topic").value), 10
        )
        self.detached_publisher = self.create_publisher(
            String, "/uav/dock/detached", 10
        )
        self.status_publisher = self.create_publisher(
            String, "/uav/dock/hardware_status", 10
        )
        self.create_subscription(
            Empty,
            str(self.get_parameter("attach_topic").value),
            self.on_attach,
            10,
        )
        self.create_subscription(
            Empty,
            str(self.get_parameter("detach_topic").value),
            self.on_detach,
            10,
        )
        self.create_subscription(String, "/uav/mavlink/status", self.on_telemetry, 10)
        self.create_subscription(String, "/mission/status", self.on_mission, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_ugv_odom, 20)
        self.create_subscription(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            self.on_emergency_stop,
            10,
        )
        for key in self.feedback:
            self.create_subscription(
                Bool,
                str(self.get_parameter(f"{key}_topic").value),
                lambda message, channel=key: self.on_feedback(channel, message),
                10,
            )
        self.create_service(SetBool, "~/enable", self.on_enable)
        self.timer = create_steady_timer(self, 0.05, self.on_timer)

    def on_telemetry(self, message: String) -> None:
        self.telemetry = _json(message)
        self.telemetry_time = time.monotonic()

    def on_mission(self, message: String) -> None:
        self.mission = _json(message)
        self.mission_time = time.monotonic()

    def on_ugv_odom(self, message: Odometry) -> None:
        velocity = message.twist.twist.linear
        self.ugv_speed = math.hypot(float(velocity.x), float(velocity.y))
        self.ugv_yaw_rate = float(message.twist.twist.angular.z)
        self.ugv_time = time.monotonic()

    def on_feedback(self, key: str, message: Bool) -> None:
        self.feedback[key] = bool(message.data)
        self.feedback_times[key] = time.monotonic()

    def on_emergency_stop(self, message: Bool) -> None:
        self.emergency_stop = bool(message.data)
        self.emergency_stop_time = time.monotonic()
        if self.emergency_stop:
            # Reset is deliberately two-step: clearing the system latch does
            # not silently restore authority to a physical actuator.
            self.command_enabled = False
            self.last_result = "disabled_by_emergency_stop"

    def on_enable(self, request: SetBool.Request, response: SetBool.Response):
        if not bool(request.data):
            self.command_enabled = False
            response.success = True
            response.message = (
                "Docking hardware commands disabled; existing latch state preserved"
            )
            return response
        now = time.monotonic()
        contact_state = dual_channel_state(
            self.feedback["contact_a"], self.feedback["contact_b"]
        )
        locked_state = dual_channel_state(
            self.feedback["locked_a"], self.feedback["locked_b"]
        )
        safe = (
            not self.emergency_stop
            and self._state_fresh(now)
            and self._feedback_fresh(now)
            and feedback_safe_for_enable(
                contact_state=contact_state, locked_state=locked_state
            )
        )
        self.command_enabled = bool(safe)
        response.success = bool(safe)
        response.message = (
            "Docking hardware commands enabled"
            if safe
            else "Enable rejected: safety state or redundant feedback is not ready"
        )
        return response

    def _state_fresh(self, now: float) -> bool:
        return all(
            timestamp > 0.0 and now - timestamp <= self.state_timeout
            for timestamp in (
                self.telemetry_time,
                self.mission_time,
                self.ugv_time,
                self.emergency_stop_time,
            )
        )

    def _feedback_fresh(self, now: float) -> bool:
        return all(
            timestamp > 0.0 and now - timestamp <= self.feedback_timeout
            for timestamp in self.feedback_times.values()
        )

    def _publish_lock_command(self, locked: bool, request_name: str) -> None:
        locked_feedback = dual_channel_state(
            self.feedback["locked_a"], self.feedback["locked_b"]
        )
        now = time.monotonic()
        should_publish, operation_started = operation_command_decision(
            commanded_locked=self.commanded_locked,
            requested_locked=locked,
            feedback_locked=locked_feedback,
            operation_started_s=self.operation_started,
            now_s=now,
        )
        self.commanded_locked = bool(locked)
        self.operation_started = operation_started
        self.last_request = request_name
        if not should_publish:
            self.last_result = "already_confirmed"
            return
        command = Bool()
        command.data = bool(locked)
        self.lock_publisher.publish(command)
        self.last_result = (
            "command_repeated_timeout_preserved"
            if operation_started < now
            else "commanded"
        )

    def on_attach(self, _message: Empty) -> None:
        now = time.monotonic()
        self.last_request = "attach"
        if not self.command_enabled:
            self.last_result = "rejected_disabled"
            return
        if self.emergency_stop:
            self.last_result = "rejected_emergency_stop"
            return
        if not self._state_fresh(now) or not self._feedback_fresh(now):
            self.last_result = "rejected_stale_state_or_feedback"
            return
        altitude = float(self.telemetry.get("relative_alt_m") or 0.0)
        if not physical_attach_authorized(
            mission_state=str(self.mission.get("state", "")),
            contact_a=self.feedback["contact_a"],
            contact_b=self.feedback["contact_b"],
            armed=bool(self.telemetry.get("armed", False)),
            landed=self.telemetry.get("landed"),
            autopilot_mode=str(self.telemetry.get("mode", "")),
            altitude_m=altitude,
            ugv_speed_mps=self.ugv_speed,
            ugv_yaw_rate_rps=self.ugv_yaw_rate,
            stationary_speed_limit_mps=self.stationary_speed_limit,
            moving_speed_limit_mps=self.moving_speed_limit,
            moving_yaw_rate_limit_rps=self.moving_yaw_rate_limit,
            moving_capture_max_altitude_m=self.moving_capture_max_altitude,
        ):
            self.last_result = "rejected_interlock"
            return
        self._publish_lock_command(True, "attach")

    def on_detach(self, _message: Empty) -> None:
        now = time.monotonic()
        self.last_request = "detach"
        if not self.command_enabled:
            self.last_result = "rejected_disabled"
            return
        if self.emergency_stop:
            self.last_result = "rejected_emergency_stop"
            return
        if not self._state_fresh(now) or not self._feedback_fresh(now):
            self.last_result = "rejected_stale_state_or_feedback"
            return
        if not physical_release_authorized(
            mission_state=str(self.mission.get("state", "")),
            armed=bool(self.telemetry.get("armed", False)),
            landed=self.telemetry.get("landed"),
            ugv_speed_mps=self.ugv_speed,
            stationary_speed_limit_mps=self.stationary_speed_limit,
        ):
            self.last_result = "rejected_interlock"
            return
        self._publish_lock_command(False, "detach")

    def on_timer(self) -> None:
        now = time.monotonic()
        feedback_fresh = self._feedback_fresh(now)
        locked_state = (
            dual_channel_state(
                self.feedback["locked_a"], self.feedback["locked_b"]
            )
            if feedback_fresh
            else None
        )
        contact_state = (
            dual_channel_state(
                self.feedback["contact_a"], self.feedback["contact_b"]
            )
            if feedback_fresh
            else None
        )
        critical_fault = ""
        if not feedback_fresh and now - self.boot_time > self.startup_grace:
            critical_fault = "DOCK_FEEDBACK_TIMEOUT"
        elif locked_state is None or contact_state is None:
            critical_fault = "DOCK_REDUNDANT_CHANNEL_DISAGREEMENT"
        elif locked_state and not contact_state:
            critical_fault = "DOCK_CONTACT_LOST_WHILE_LOCKED"
        elif (
            self.commanded_locked is not None
            and locked_state != self.commanded_locked
            and self.operation_started > 0.0
            and now - self.operation_started > self.operation_timeout
        ):
            critical_fault = "DOCK_OPERATION_TIMEOUT"
        elif (
            self.commanded_locked is not None
            and locked_state != self.commanded_locked
            and self.operation_started == 0.0
        ):
            critical_fault = "DOCK_UNEXPECTED_STATE_CHANGE"
        elif (
            self.commanded_locked is not None
            and locked_state == self.commanded_locked
        ):
            self.operation_started = 0.0
            self.last_result = "confirmed"

        if feedback_fresh and locked_state is not None:
            detached = String()
            detached.data = "attached" if locked_state else "detached"
            self.detached_publisher.publish(detached)

        status = String()
        status.data = json.dumps(
            {
                "schema_version": "1.0",
                "enabled": self.command_enabled,
                "healthy": bool(feedback_fresh and not critical_fault),
                "critical_fault": critical_fault,
                "emergency_stop": self.emergency_stop,
                "feedback_fresh": feedback_fresh,
                "locked": locked_state,
                "contact": contact_state,
                "commanded_locked": self.commanded_locked,
                "operation_age_s": (
                    None
                    if self.operation_started == 0.0
                    else round(now - self.operation_started, 3)
                ),
                "last_request": self.last_request,
                "last_result": self.last_result,
                "mission_state": self.mission.get("state", "UNKNOWN"),
                "ugv_speed_mps": round(self.ugv_speed, 4),
                "ugv_yaw_rate_rps": round(self.ugv_yaw_rate, 4),
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_publisher.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DockingHardwareGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Never unlock as a shutdown side effect.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
