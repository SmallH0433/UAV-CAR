"""ROS 2 wrapper for selectable differential, Ackermann and 4WS adapters."""

import json
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool

from .chassis_adapters import make_chassis_adapter
from .motion_gate import motion_gate_open
from .protocol import clamp
from .ros_compat import run_shutdown_action
from .runtime_timing import create_steady_timer


class UgvChassisAdapter(Node):
    """Convert Nav2 Twist output into commands feasible for the selected chassis."""

    def __init__(self) -> None:
        super().__init__("ugv_chassis_adapter")
        self.declare_parameter("adapter_type", "ackermann")
        # Collision Monitor is the required upstream software safety layer.
        self.declare_parameter("input_topic", "/ugv/safety/cmd_vel")
        self.declare_parameter("output_topic", "/ugv/cmd_vel")
        self.declare_parameter("command_enabled", False)
        self.declare_parameter("command_timeout_s", 0.35)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("max_linear_mps", 0.8)
        self.declare_parameter("max_angular_rps", 1.0)
        self.declare_parameter("max_linear_accel_mps2", 0.8)
        self.declare_parameter("max_angular_accel_rps2", 1.2)
        self.declare_parameter("wheelbase_m", 0.65)
        self.declare_parameter("max_steering_angle_rad", 0.41)
        self.declare_parameter("min_linear_for_turn_mps", 0.03)
        self.declare_parameter("speed_scale_topic", "/ugv/speed_scale")
        self.declare_parameter("require_speed_gate", True)
        self.declare_parameter("speed_gate_timeout_s", 0.25)
        self.declare_parameter("emergency_stop_topic", "/system/emergency_stop")

        self.adapter_type = str(self.get_parameter("adapter_type").value)
        self.max_linear_accel = float(self.get_parameter("max_linear_accel_mps2").value)
        self.max_angular_accel = float(self.get_parameter("max_angular_accel_rps2").value)
        self.timeout = float(self.get_parameter("command_timeout_s").value)
        self.require_speed_gate = bool(self.get_parameter("require_speed_gate").value)
        self.speed_gate_timeout = float(
            self.get_parameter("speed_gate_timeout_s").value
        )
        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        if self.max_linear_accel <= 0.0 or self.max_angular_accel <= 0.0:
            raise ValueError("adapter acceleration limits must be positive")
        if self.timeout <= 0.0 or publish_rate <= 0.0 or self.speed_gate_timeout <= 0.0:
            raise ValueError("adapter timeouts and publish rate must be positive")

        self.adapter = make_chassis_adapter(
            self.adapter_type,
            float(self.get_parameter("max_linear_mps").value),
            float(self.get_parameter("max_angular_rps").value),
            float(self.get_parameter("wheelbase_m").value),
            float(self.get_parameter("max_steering_angle_rad").value),
            float(self.get_parameter("min_linear_for_turn_mps").value),
        )
        self.command_enabled = bool(self.get_parameter("command_enabled").value)
        self.requested = Twist()
        self.last_input_time = 0.0
        self.last_tick_time = time.monotonic()
        self.last_output = Twist()
        self.last_adapted = self.adapter.adapt(0.0, 0.0)
        self.reason = "waiting_command"
        self.speed_scale = 0.0
        self.last_speed_scale_time = 0.0
        self.emergency_stop = False

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.status_publisher = self.create_publisher(String, "/ugv/chassis_adapter/status", 10)
        self.subscription = self.create_subscription(Twist, input_topic, self.on_command, 10)
        self.speed_scale_subscription = self.create_subscription(
            Float64,
            str(self.get_parameter("speed_scale_topic").value),
            self.on_speed_scale,
            10,
        )
        self.emergency_subscription = self.create_subscription(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            self.on_emergency_stop,
            10,
        )
        self.enable_service = self.create_service(SetBool, "~/enable", self.on_enable)
        self.timer = create_steady_timer(self, 1.0 / publish_rate, self.on_timer)
        self.status_timer = create_steady_timer(self, 0.5, self.publish_status)
        self.get_logger().info(
            f"{self.adapter.name} adapter {input_topic} -> {output_topic}; "
            f"enabled={self.command_enabled}"
        )

    def on_command(self, message: Twist) -> None:
        self.requested = message
        self.last_input_time = time.monotonic()

    def on_speed_scale(self, message: Float64) -> None:
        self.speed_scale = clamp(float(message.data), 0.0, 1.0)
        self.last_speed_scale_time = time.monotonic()

    def on_emergency_stop(self, message: Bool) -> None:
        self.emergency_stop = bool(message.data)
        if self.emergency_stop:
            self.stop("emergency_stop")

    def on_enable(self, request: SetBool.Request, response: SetBool.Response):
        self.command_enabled = bool(request.data)
        if not self.command_enabled:
            self.stop("disabled")
        response.success = True
        response.message = (
            f"{self.adapter.name} adapter enabled"
            if self.command_enabled
            else "adapter disabled and command zeroed"
        )
        return response

    def stop(self, reason: str) -> None:
        self.last_output = Twist()
        self.last_adapted = self.adapter.adapt(0.0, 0.0)
        self.reason = reason
        self.publisher.publish(self.last_output)

    def on_timer(self) -> None:
        now = time.monotonic()
        dt = max(1.0e-3, min(0.2, now - self.last_tick_time))
        self.last_tick_time = now
        fresh = self.last_input_time > 0.0 and now - self.last_input_time <= self.timeout
        gate_age = (
            None
            if self.last_speed_scale_time == 0.0
            else now - self.last_speed_scale_time
        )
        gate_open = motion_gate_open(
            command_enabled=self.command_enabled,
            emergency_stop=self.emergency_stop,
            require_gate=self.require_speed_gate,
            gate_value=self.speed_scale,
            gate_age_s=gate_age,
            gate_timeout_s=self.speed_gate_timeout,
        )
        if self.emergency_stop:
            self.stop("emergency_stop")
            return
        if not self.command_enabled:
            self.stop("disabled")
            return
        if not gate_open:
            self.stop(
                "speed_gate_timeout"
                if self.require_speed_gate
                and gate_age is not None
                and gate_age > self.speed_gate_timeout
                else "speed_gate_closed"
            )
            return
        if not fresh:
            self.stop("command_timeout")
            return

        # Scale linear and angular velocity together so Ackermann curvature is
        # preserved while the mission slows the vehicle for dynamic docking.
        target = self.adapter.adapt(
            self.requested.linear.x * self.speed_scale,
            self.requested.angular.z * self.speed_scale,
        )
        linear_step = self.max_linear_accel * dt
        angular_step = self.max_angular_accel * dt
        ramped_linear = self.last_output.linear.x + clamp(
            target.linear_mps - self.last_output.linear.x, -linear_step, linear_step
        )
        ramped_angular = self.last_output.angular.z + clamp(
            target.angular_rps - self.last_output.angular.z, -angular_step, angular_step
        )
        safe = self.adapter.adapt(ramped_linear, ramped_angular)
        output = Twist()
        output.linear.x = safe.linear_mps
        output.angular.z = safe.angular_rps
        self.last_output = output
        self.last_adapted = safe
        self.reason = safe.reason
        self.publisher.publish(output)

    def publish_status(self) -> None:
        now = time.monotonic()
        gate_age = (
            None
            if self.last_speed_scale_time == 0.0
            else now - self.last_speed_scale_time
        )
        gate_open = motion_gate_open(
            command_enabled=self.command_enabled,
            emergency_stop=self.emergency_stop,
            require_gate=self.require_speed_gate,
            gate_value=self.speed_scale,
            gate_age_s=gate_age,
            gate_timeout_s=self.speed_gate_timeout,
        )
        status = {
            "schema_version": "1.0",
            "adapter_type": self.adapter.name,
            "enabled": self.command_enabled,
            "emergency_stop": self.emergency_stop,
            "reason": self.reason,
            "input_age_s": None
            if self.last_input_time == 0.0
            else round(now - self.last_input_time, 3),
            "requested_linear_mps": round(self.requested.linear.x, 4),
            "requested_angular_rps": round(self.requested.angular.z, 4),
            "speed_scale": round(self.speed_scale, 3),
            "speed_gate_required": self.require_speed_gate,
            "speed_gate_open": gate_open,
            "speed_gate_age_s": None if gate_age is None else round(gate_age, 3),
            "speed_gate_timeout_s": round(self.speed_gate_timeout, 3),
            "output_linear_mps": round(self.last_output.linear.x, 4),
            "output_angular_rps": round(self.last_output.angular.z, 4),
            "curvature_per_m": round(self.last_adapted.curvature_per_m, 4),
            "equivalent_steering_angle_rad": round(
                self.last_adapted.steering_angle_rad, 4
            ),
            "saturated": self.last_adapted.saturated,
        }
        message = String()
        message.data = json.dumps(status, ensure_ascii=False, sort_keys=True)
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UgvChassisAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            run_shutdown_action(lambda: node.stop("shutdown"))
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
