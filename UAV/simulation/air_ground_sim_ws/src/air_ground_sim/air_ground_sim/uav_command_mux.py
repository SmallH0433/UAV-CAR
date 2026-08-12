"""Priority and watchdog mux for UAV navigation and AprilTag-follow commands."""

import json
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from .ros_compat import run_shutdown_action
from .runtime_timing import create_steady_timer


class UavCommandMux(Node):
    """Give docking, then guarded visual following, priority over navigation."""

    def __init__(self) -> None:
        super().__init__("uav_command_mux")
        self.declare_parameter("navigation_topic", "/uav/nav/cmd_vel")
        self.declare_parameter("follow_topic", "/uav/follow/cmd_vel")
        self.declare_parameter("docking_topic", "/uav/docking/cmd_vel")
        self.declare_parameter("output_topic", "/uav/cmd_vel")
        self.declare_parameter("navigation_status_topic", "/uav/navigation/status")
        self.declare_parameter("follow_status_topic", "/apriltag/status")
        self.declare_parameter("docking_status_topic", "/uav/docking/status")
        self.declare_parameter("command_timeout_s", 0.35)
        self.declare_parameter("status_timeout_s", 1.0)
        self.declare_parameter("command_enabled", False)
        self.declare_parameter("emergency_stop_topic", "/system/emergency_stop")

        self.command_timeout = float(self.get_parameter("command_timeout_s").value)
        self.status_timeout = float(self.get_parameter("status_timeout_s").value)
        self.command_enabled = bool(self.get_parameter("command_enabled").value)
        self.navigation_command = Twist()
        self.follow_command = Twist()
        self.docking_command = Twist()
        self.navigation_command_time = 0.0
        self.follow_command_time = 0.0
        self.docking_command_time = 0.0
        self.navigation_status_time = 0.0
        self.follow_status_time = 0.0
        self.docking_status_time = 0.0
        self.navigation_active = False
        self.follow_active = False
        self.docking_active = False
        self.mode = "stopped"
        self.emergency_stop = False

        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self.status_publisher = self.create_publisher(String, "/uav/command_mux/status", 10)
        self.create_subscription(
            Twist,
            str(self.get_parameter("navigation_topic").value),
            self.on_navigation_command,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            self.on_emergency_stop,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("follow_topic").value),
            self.on_follow_command,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("docking_topic").value),
            self.on_docking_command,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("navigation_status_topic").value),
            self.on_navigation_status,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("follow_status_topic").value),
            self.on_follow_status,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("docking_status_topic").value),
            self.on_docking_status,
            10,
        )
        self.enable_service = self.create_service(SetBool, "~/enable", self.on_enable)
        self.timer = create_steady_timer(self, 0.05, self.on_timer)
        self.status_timer = create_steady_timer(self, 0.5, self.publish_status)

    def on_navigation_command(self, message: Twist) -> None:
        self.navigation_command = message
        self.navigation_command_time = time.monotonic()

    def on_follow_command(self, message: Twist) -> None:
        self.follow_command = message
        self.follow_command_time = time.monotonic()

    def on_docking_command(self, message: Twist) -> None:
        self.docking_command = message
        self.docking_command_time = time.monotonic()

    def _parse_active(self, message: String):
        try:
            return bool(json.loads(message.data).get("active", False))
        except (json.JSONDecodeError, AttributeError):
            return False

    def on_navigation_status(self, message: String) -> None:
        self.navigation_active = self._parse_active(message)
        self.navigation_status_time = time.monotonic()

    def on_follow_status(self, message: String) -> None:
        self.follow_active = self._parse_active(message)
        self.follow_status_time = time.monotonic()

    def on_docking_status(self, message: String) -> None:
        self.docking_active = self._parse_active(message)
        self.docking_status_time = time.monotonic()

    def on_emergency_stop(self, message: Bool) -> None:
        self.emergency_stop = bool(message.data)
        if self.emergency_stop:
            self.publisher.publish(Twist())
            self.mode = "emergency_stop"

    def on_enable(self, request: SetBool.Request, response: SetBool.Response):
        self.command_enabled = bool(request.data)
        if self.emergency_stop:
            self.mode = "emergency_stop"
        elif not self.command_enabled:
            self.publisher.publish(Twist())
            self.mode = "disabled"
        response.success = True
        response.message = "UAV command mux enabled" if request.data else "UAV commands stopped"
        return response

    def on_timer(self) -> None:
        now = time.monotonic()
        output = Twist()
        self.mode = "stopped"
        if not self.command_enabled:
            self.mode = "disabled"
        elif (
            self.docking_active
            and now - self.docking_status_time <= self.status_timeout
            and now - self.docking_command_time <= self.command_timeout
        ):
            output = self.docking_command
            self.mode = "docking"
        elif (
            self.follow_active
            and now - self.follow_status_time <= self.status_timeout
            and now - self.follow_command_time <= self.command_timeout
        ):
            output = self.follow_command
            self.mode = "apriltag_follow"
        elif (
            self.navigation_active
            and now - self.navigation_status_time <= self.status_timeout
            and now - self.navigation_command_time <= self.command_timeout
        ):
            output = self.navigation_command
            self.mode = "navigation"
        self.publisher.publish(output)

    def publish_status(self) -> None:
        status = {
            "schema_version": "1.0",
            "enabled": self.command_enabled,
            "emergency_stop": self.emergency_stop,
            "mode": self.mode,
            "navigation_active": self.navigation_active,
            "follow_active": self.follow_active,
            "docking_active": self.docking_active,
        }
        message = String()
        message.data = json.dumps(status, ensure_ascii=False, sort_keys=True)
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UavCommandMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            run_shutdown_action(lambda: node.publisher.publish(Twist()))
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
