"""Bounded ROS 2 command interface for the simulated or physical UAV gimbal."""

import json
import math

from geometry_msgs.msg import Vector3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

from .protocol import clamp
from .runtime_timing import create_steady_timer


class UavGimbalController(Node):
    def __init__(self) -> None:
        super().__init__("uav_gimbal_controller")
        self.declare_parameter("setpoint_topic", "/uav/gimbal/setpoint")
        self.declare_parameter("yaw_command_topic", "/uav/gimbal/cmd_yaw")
        self.declare_parameter("pitch_command_topic", "/uav/gimbal/cmd_pitch")
        self.declare_parameter("yaw_min_rad", -2.967)
        self.declare_parameter("yaw_max_rad", 2.967)
        self.declare_parameter("pitch_min_rad", -0.436)
        self.declare_parameter("pitch_max_rad", 1.5708)
        self.declare_parameter("rate_limit_rad_s", 1.4)

        self.yaw_min = float(self.get_parameter("yaw_min_rad").value)
        self.yaw_max = float(self.get_parameter("yaw_max_rad").value)
        self.pitch_min = float(self.get_parameter("pitch_min_rad").value)
        self.pitch_max = float(self.get_parameter("pitch_max_rad").value)
        self.rate_limit = max(float(self.get_parameter("rate_limit_rad_s").value), 0.05)
        self.target_yaw = 0.0
        self.target_pitch = 0.0
        self.command_yaw = 0.0
        self.command_pitch = 0.0
        self.mode = "forward"

        self.yaw_publisher = self.create_publisher(
            Float64, str(self.get_parameter("yaw_command_topic").value), 10
        )
        self.pitch_publisher = self.create_publisher(
            Float64, str(self.get_parameter("pitch_command_topic").value), 10
        )
        self.status_publisher = self.create_publisher(String, "/uav/gimbal/status", 10)
        self.create_subscription(
            Vector3,
            str(self.get_parameter("setpoint_topic").value),
            self.on_setpoint,
            10,
        )
        self.create_service(Trigger, "~/center", self.on_center)
        self.create_service(Trigger, "~/look_down", self.on_look_down)
        self.timer = create_steady_timer(self, 0.02, self.control_tick)
        self.status_timer = create_steady_timer(self, 0.5, self.publish_status)

    def on_setpoint(self, message: Vector3) -> None:
        self.target_yaw = clamp(float(message.x), self.yaw_min, self.yaw_max)
        self.target_pitch = clamp(float(message.y), self.pitch_min, self.pitch_max)
        self.mode = "manual_setpoint"

    def on_center(self, _request: Trigger.Request, response: Trigger.Response):
        self.target_yaw = 0.0
        self.target_pitch = 0.0
        self.mode = "forward"
        response.success = True
        response.message = "Gimbal moving to forward preset"
        return response

    def on_look_down(self, _request: Trigger.Request, response: Trigger.Response):
        self.target_yaw = 0.0
        self.target_pitch = math.pi / 2.0
        self.mode = "landing_down"
        response.success = True
        response.message = "Gimbal moving to landing preset"
        return response

    def _slew(self, current: float, target: float, dt: float) -> float:
        maximum_step = self.rate_limit * dt
        return current + clamp(target - current, -maximum_step, maximum_step)

    def control_tick(self) -> None:
        self.command_yaw = self._slew(self.command_yaw, self.target_yaw, 0.02)
        self.command_pitch = self._slew(self.command_pitch, self.target_pitch, 0.02)
        yaw = Float64()
        yaw.data = self.command_yaw
        pitch = Float64()
        pitch.data = self.command_pitch
        self.yaw_publisher.publish(yaw)
        self.pitch_publisher.publish(pitch)

    def publish_status(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                "mode": self.mode,
                "target_rad": [round(self.target_yaw, 4), round(self.target_pitch, 4)],
                "command_rad": [round(self.command_yaw, 4), round(self.command_pitch, 4)],
                "at_target": abs(self.target_yaw - self.command_yaw) < 0.02
                and abs(self.target_pitch - self.command_pitch) < 0.02,
                "feedback_source": "command_estimate",
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UavGimbalController()
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
