"""Rate-limited, watchdog-protected command gateway for simulated or real UGVs."""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from .protocol import clamp
from .runtime_timing import create_steady_timer


class UgvCommandGateway(Node):
    """Apply a safety envelope before forwarding vehicle velocity commands."""

    def __init__(self) -> None:
        super().__init__("ugv_command_gateway")
        self.declare_parameter("input_topic", "/ugv/cmd_vel")
        self.declare_parameter("output_topic", "/ugv/sim/cmd_vel")
        self.declare_parameter("command_enabled", False)
        self.declare_parameter("max_linear_mps", 1.0)
        self.declare_parameter("max_angular_rps", 1.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("emergency_stop_topic", "/system/emergency_stop")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.command_enabled = bool(self.get_parameter("command_enabled").value)
        self.max_linear = float(self.get_parameter("max_linear_mps").value)
        self.max_angular = float(self.get_parameter("max_angular_rps").value)
        self.timeout = float(self.get_parameter("command_timeout_s").value)
        self.last_command_time = 0.0
        self.latest = Twist()
        self.last_output = Twist()
        self.emergency_stop = False
        self.reason = "waiting_command"

        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.subscription = self.create_subscription(Twist, input_topic, self.on_command, 10)
        self.emergency_subscription = self.create_subscription(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            self.on_emergency_stop,
            10,
        )
        self.status_publisher = self.create_publisher(
            String, "/ugv/command_gateway/status", 10
        )
        self.enable_service = self.create_service(SetBool, "~/enable", self.on_enable)
        self.timer = create_steady_timer(self, 0.05, self.on_timer)
        self.status_timer = create_steady_timer(self, 0.5, self.publish_status)
        self.get_logger().info(
            f"UGV gateway {input_topic} -> {output_topic}; enabled={self.command_enabled}"
        )

    def on_command(self, message: Twist) -> None:
        safe = Twist()
        safe.linear.x = clamp(message.linear.x, -self.max_linear, self.max_linear)
        safe.angular.z = clamp(message.angular.z, -self.max_angular, self.max_angular)
        self.latest = safe
        self.last_command_time = time.monotonic()

    def on_emergency_stop(self, message: Bool) -> None:
        self.emergency_stop = bool(message.data)
        if self.emergency_stop:
            self._publish_stop("emergency_stop")

    def on_enable(self, request: SetBool.Request, response: SetBool.Response):
        self.command_enabled = bool(request.data)
        if not self.command_enabled:
            self.latest = Twist()
            self._publish_stop("disabled")
        response.success = True
        response.message = "UGV commands enabled" if request.data else "UGV stopped"
        return response

    def on_timer(self) -> None:
        fresh = (time.monotonic() - self.last_command_time) <= self.timeout
        if self.emergency_stop:
            self._publish_stop("emergency_stop")
        elif not self.command_enabled:
            self._publish_stop("disabled")
        elif not fresh:
            self._publish_stop("command_timeout")
        else:
            self.last_output = self.latest
            self.reason = "forwarding"
            self.publisher.publish(self.last_output)

    def _publish_stop(self, reason: str) -> None:
        self.last_output = Twist()
        self.reason = reason
        self.publisher.publish(self.last_output)

    def publish_status(self) -> None:
        age = (
            None
            if self.last_command_time == 0.0
            else round(time.monotonic() - self.last_command_time, 3)
        )
        message = String()
        message.data = json.dumps(
            {
                "schema_version": "1.0",
                "enabled": self.command_enabled,
                "emergency_stop": self.emergency_stop,
                "reason": self.reason,
                "input_age_s": age,
                "command_timeout_s": self.timeout,
                "output_linear_mps": round(self.last_output.linear.x, 4),
                "output_angular_rps": round(self.last_output.angular.z, 4),
            },
            sort_keys=True,
        )
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UgvCommandGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.publisher.publish(Twist())
            except Exception as error:  # Context may already be invalid during launch shutdown.
                node.get_logger().debug(f"Unable to publish final stop command: {error}")
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
