"""Fail-closed UGV control-authority mux for autonomy and operator commands.

Ported from the air_ground_sim project. Adds the ``require_mission_status``
parameter (default ``false``): when false the mux does not subscribe to or
require ``/mission/status`` and ``/ugv/mission_gate``; arbitration is then a
simple teleop-over-autonomy scheme — a fresh operator heartbeat (0.6 s) lets
``/ugv/teleop/cmd_vel`` override the autonomous ``/cmd_vel`` input, and
without a heartbeat the autonomous command (avoidance node) passes through.

Optional ``steering_assist`` parameter (default ``false``, dynamically
settable): when true and both operator and autonomous inputs are fresh, the
mux blends instead of overriding — angular (steering) comes from teleop while
linear keeps the autonomous cruise speed, unless the operator explicitly
commands a non-zero linear (then teleop linear wins, allowing braking).
The web gateway flips this on while avoidance cruise is enabled.
"""

import json
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool

from .protocol import clamp
from .ros_compat import run_shutdown_action
from .runtime_timing import create_steady_timer


class UgvControlMux(Node):
    """Select exactly one command authority and heartbeat the downstream gate."""

    def __init__(self) -> None:
        super().__init__("ugv_control_mux")
        self.declare_parameter("navigation_topic", "/cmd_vel")
        self.declare_parameter("teleop_topic", "/ugv/teleop/cmd_vel")
        self.declare_parameter("output_topic", "/ugv/control/cmd_vel")
        self.declare_parameter("mission_status_topic", "/mission/status")
        self.declare_parameter("mission_gate_topic", "/ugv/mission_gate")
        self.declare_parameter("operator_heartbeat_topic", "/ugv/operator/heartbeat")
        self.declare_parameter("speed_gate_topic", "/ugv/speed_scale")
        self.declare_parameter("emergency_stop_topic", "/system/emergency_stop")
        self.declare_parameter("require_mission_status", False)
        self.declare_parameter("command_enabled", False)
        self.declare_parameter("command_timeout_s", 0.35)
        self.declare_parameter("mission_status_timeout_s", 0.75)
        self.declare_parameter("mission_gate_timeout_s", 0.50)
        self.declare_parameter("operator_timeout_s", 0.60)
        self.declare_parameter("steering_assist", False)
        self.declare_parameter("publish_rate_hz", 30.0)

        self.command_enabled = bool(self.get_parameter("command_enabled").value)
        self.require_mission_status = bool(
            self.get_parameter("require_mission_status").value
        )
        self.command_timeout = max(float(self.get_parameter("command_timeout_s").value), 0.05)
        self.mission_status_timeout = max(
            float(self.get_parameter("mission_status_timeout_s").value), 0.05
        )
        self.mission_gate_timeout = max(
            float(self.get_parameter("mission_gate_timeout_s").value), 0.05
        )
        self.operator_timeout = max(
            float(self.get_parameter("operator_timeout_s").value), 0.05
        )
        rate = max(float(self.get_parameter("publish_rate_hz").value), 1.0)

        self.navigation_command = Twist()
        self.teleop_command = Twist()
        self.navigation_time = 0.0
        self.teleop_time = 0.0
        self.mission_status_time = 0.0
        self.mission_gate_time = 0.0
        self.operator_time = 0.0
        self.mission_active = False
        self.mission_paused = False
        self.mission_state = "UNKNOWN"
        self.mission_gate = 0.0
        self.operator_requested = False
        self.emergency_stop = False
        self.authority = "none"
        self.reason = "waiting_for_authority"
        self.output = Twist()
        self.gate_open = False

        self.command_publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self.gate_publisher = self.create_publisher(
            Float64, str(self.get_parameter("speed_gate_topic").value), 10
        )
        self.status_publisher = self.create_publisher(String, "/ugv/control_mux/status", 10)
        self.create_subscription(
            Twist,
            str(self.get_parameter("navigation_topic").value),
            self.on_navigation,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("teleop_topic").value),
            self.on_teleop,
            10,
        )
        if self.require_mission_status:
            self.create_subscription(
                String,
                str(self.get_parameter("mission_status_topic").value),
                self.on_mission_status,
                10,
            )
            self.create_subscription(
                Float64,
                str(self.get_parameter("mission_gate_topic").value),
                self.on_mission_gate,
                10,
            )
        self.create_subscription(
            Bool,
            str(self.get_parameter("operator_heartbeat_topic").value),
            self.on_operator_heartbeat,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            self.on_emergency_stop,
            10,
        )
        self.enable_service = self.create_service(SetBool, "~/enable", self.on_enable)
        self.timer = create_steady_timer(self, 1.0 / rate, self.on_timer)
        self.status_timer = create_steady_timer(self, 0.25, self.publish_status)

    def on_navigation(self, message: Twist) -> None:
        self.navigation_command = message
        self.navigation_time = time.monotonic()

    def on_teleop(self, message: Twist) -> None:
        self.teleop_command = message
        self.teleop_time = time.monotonic()

    def on_mission_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            status = {}
        self.mission_active = bool(status.get("active", False))
        self.mission_paused = bool(status.get("paused", False))
        self.mission_state = str(status.get("state", "UNKNOWN"))
        self.mission_status_time = time.monotonic()

    def on_mission_gate(self, message: Float64) -> None:
        self.mission_gate = clamp(float(message.data), 0.0, 1.0)
        self.mission_gate_time = time.monotonic()

    def on_operator_heartbeat(self, message: Bool) -> None:
        self.operator_requested = bool(message.data)
        self.operator_time = time.monotonic()

    def on_emergency_stop(self, message: Bool) -> None:
        self.emergency_stop = bool(message.data)
        if self.emergency_stop:
            self._publish_stop("emergency_stop")

    def on_enable(self, request: SetBool.Request, response: SetBool.Response):
        self.command_enabled = bool(request.data)
        if not self.command_enabled:
            self._publish_stop("disabled")
        response.success = True
        response.message = "UGV control mux enabled" if request.data else "UGV control mux disabled and stopped"
        return response

    def _publish(self, command: Twist, gate_open: bool, authority: str, reason: str) -> None:
        self.output = command
        self.gate_open = bool(gate_open)
        self.authority = authority
        self.reason = reason
        gate = Float64()
        gate.data = 1.0 if self.gate_open else 0.0
        # Close the hardware gate before publishing a stop; open it only after
        # a valid selected command has been established.
        if self.gate_open:
            self.command_publisher.publish(command)
            self.gate_publisher.publish(gate)
        else:
            self.gate_publisher.publish(gate)
            self.command_publisher.publish(command)

    def _publish_stop(self, reason: str) -> None:
        self._publish(Twist(), False, "none", reason)

    def on_timer(self) -> None:
        now = time.monotonic()
        if not self.command_enabled:
            self._publish_stop("disabled")
            return
        if self.emergency_stop:
            self._publish_stop("emergency_stop")
            return
        if self.require_mission_status:
            self._arbitrate_with_mission(now)
        else:
            self._arbitrate_teleop_over_autonomy(now)

    def _arbitrate_with_mission(self, now: float) -> None:
        """Original air-ground behaviour: a fresh mission status is mandatory."""
        mission_status_fresh = (
            self.mission_status_time > 0.0
            and now - self.mission_status_time <= self.mission_status_timeout
        )
        if not mission_status_fresh:
            self._publish_stop("mission_status_timeout")
            return

        if self.mission_active and not self.mission_paused:
            gate_fresh = (
                self.mission_gate_time > 0.0
                and now - self.mission_gate_time <= self.mission_gate_timeout
            )
            command_fresh = (
                self.navigation_time > 0.0
                and now - self.navigation_time <= self.command_timeout
            )
            if not gate_fresh:
                self._publish_stop("mission_gate_timeout")
            elif self.mission_gate <= 0.0:
                self._publish_stop("mission_gate_closed")
            elif not command_fresh:
                self._publish_stop("navigation_command_timeout")
            else:
                self._publish(self.navigation_command, True, "mission", "mission_navigation")
            return

        self._arbitrate_teleop_over_autonomy(now, operator_required=True)

    def _arbitrate_teleop_over_autonomy(
        self, now: float, operator_required: bool = False
    ) -> None:
        """Teleop overrides autonomy while the operator heartbeat is fresh.

        With ``operator_required`` false (the default mission-less mode) a fresh
        autonomous ``/cmd_vel`` is forwarded whenever the operator is absent.
        """
        operator_fresh = (
            self.operator_requested
            and self.operator_time > 0.0
            and now - self.operator_time <= self.operator_timeout
        )
        teleop_fresh = (
            self.teleop_time > 0.0
            and now - self.teleop_time <= self.command_timeout
        )
        navigation_fresh = (
            self.navigation_time > 0.0
            and now - self.navigation_time <= self.command_timeout
        )

        if operator_fresh:
            steering_assist = bool(self.get_parameter("steering_assist").value)
            if steering_assist and teleop_fresh and navigation_fresh:
                # 定速巡航转向辅助：方向听操作员，速度保持巡航；
                # 操作员明确给非零线速度（刹车/加速）时以遥控为准。
                blended = Twist()
                blended.linear.x = (
                    self.teleop_command.linear.x
                    if abs(self.teleop_command.linear.x) > 1e-3
                    else self.navigation_command.linear.x
                )
                blended.angular.z = self.teleop_command.angular.z
                self._publish(blended, True, "operator_steering", "cruise_steering_assist")
            elif teleop_fresh:
                self._publish(self.teleop_command, True, "operator_teleop", "operator_teleop")
            elif navigation_fresh:
                self._publish(self.navigation_command, True, "operator_nav2", "operator_navigation")
            else:
                self._publish_stop("operator_command_timeout")
            return

        if operator_required:
            self._publish_stop("operator_heartbeat_timeout")
            return

        if navigation_fresh:
            self._publish(self.navigation_command, True, "navigation", "autonomous_navigation")
        else:
            self._publish_stop("no_active_authority")

    def publish_status(self) -> None:
        now = time.monotonic()

        def age(timestamp: float):
            return None if timestamp == 0.0 else round(now - timestamp, 3)

        message = String()
        message.data = json.dumps(
            {
                "schema_version": "1.0",
                "enabled": self.command_enabled,
                "require_mission_status": self.require_mission_status,
                "emergency_stop": self.emergency_stop,
                "authority": self.authority,
                "reason": self.reason,
                "gate_open": self.gate_open,
                "mission_active": self.mission_active,
                "mission_paused": self.mission_paused,
                "mission_state": self.mission_state,
                "mission_gate": round(self.mission_gate, 3),
                "operator_requested": self.operator_requested,
                "ages_s": {
                    "navigation": age(self.navigation_time),
                    "teleop": age(self.teleop_time),
                    "mission_status": age(self.mission_status_time),
                    "mission_gate": age(self.mission_gate_time),
                    "operator": age(self.operator_time),
                },
                "output_linear_mps": round(self.output.linear.x, 4),
                "output_angular_rps": round(self.output.angular.z, 4),
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_publisher.publish(message)

    def destroy_node(self):
        if rclpy.ok():
            run_shutdown_action(lambda: self._publish_stop("shutdown"))
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UgvControlMux()
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


if __name__ == "__main__":
    main()
