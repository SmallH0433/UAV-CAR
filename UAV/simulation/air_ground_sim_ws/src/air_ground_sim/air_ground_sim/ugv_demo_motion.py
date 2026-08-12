"""A service-gated slow vehicle path for the AprilTag following experiment."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from .ros_compat import run_shutdown_action
from .runtime_timing import create_steady_timer


class UgvDemoMotion(Node):
    def __init__(self) -> None:
        super().__init__("ugv_demo_motion")
        self.declare_parameter("command_topic", "/ugv/teleop/cmd_vel")
        self.declare_parameter("operator_heartbeat_topic", "/ugv/operator/heartbeat")
        self.declare_parameter("linear_mps", 0.20)
        self.declare_parameter("turn_rps", 0.22)
        self.declare_parameter("straight_s", 10.0)
        self.declare_parameter("turn_s", 7.0)
        self.enabled = False
        self.started = 0.0
        self.linear = float(self.get_parameter("linear_mps").value)
        self.turn = float(self.get_parameter("turn_rps").value)
        self.straight_s = float(self.get_parameter("straight_s").value)
        self.turn_s = float(self.get_parameter("turn_s").value)
        topic = str(self.get_parameter("command_topic").value)
        self.publisher = self.create_publisher(Twist, topic, 10)
        self.heartbeat_publisher = self.create_publisher(
            Bool, str(self.get_parameter("operator_heartbeat_topic").value), 10
        )
        self.service = self.create_service(SetBool, "~/enable", self.on_enable)
        self.timer = create_steady_timer(self, 0.1, self.tick)

    def on_enable(self, request: SetBool.Request, response: SetBool.Response):
        self.enabled = bool(request.data)
        self.started = time.monotonic()
        self.publish_heartbeat(self.enabled)
        if not self.enabled:
            self.publisher.publish(Twist())
        response.success = True
        response.message = "UGV demo moving" if self.enabled else "UGV demo stopped"
        return response

    def tick(self) -> None:
        # Do not compete with a keyboard / joystick publisher while the demo
        # path is disabled.  A single zero command is already sent in on_enable.
        if not self.enabled:
            return
        self.publish_heartbeat(True)
        command = Twist()
        cycle = 2.0 * (self.straight_s + self.turn_s)
        phase = (time.monotonic() - self.started) % cycle
        command.linear.x = self.linear
        if self.straight_s <= phase < self.straight_s + self.turn_s:
            command.angular.z = self.turn
        elif 2.0 * self.straight_s + self.turn_s <= phase:
            command.angular.z = self.turn
        self.publisher.publish(command)

    def publish_heartbeat(self, enabled: bool) -> None:
        heartbeat = Bool()
        heartbeat.data = bool(enabled)
        self.heartbeat_publisher.publish(heartbeat)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UgvDemoMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            run_shutdown_action(
                lambda: (node.publisher.publish(Twist()), node.publish_heartbeat(False))
            )
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
