"""仿真电机桥：替代实机 motor_driver，把阿克曼指令转成 Gazebo 整车速度指令。

订阅：
  /ackermann_cmd      (car_interfaces/AckermannCommand) 后轮速 + 转向角
  /ugv/wheel/odometry (nav_msgs/Odometry)               gz 桥回的里程计
发布：
  /ugv/sim/cmd_vel    (geometry_msgs/Twist)         经 ros_gz_bridge 到 gz
  /motor_feedback     (car_interfaces/MotorFeedback) 由 gz 里程计反算
参数：
  wheel_radius       (float, 0.076) 轮半径 m
  wheelbase          (float, 0.31)  轴距 m
  track_width        (float, 0.32)  后轮距 m
  max_steering_angle (float, 0.5)   前轮最大转向角 rad
  sim_voltage        (float, 24.0)  上报的仿真电池电压 V

话题契约与实机 motor_driver 完全一致：chassis_controller 的里程计走
/motor_feedback 真实反馈路径，换到实机时只需用 motor_driver 替换本节点。
gz 侧 AckermannSteering 插件直接收 Twist（舵机转角由插件内部解算），
因此本桥只做 阿克曼指令 ↔ (v, w) 的换算。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from car_interfaces.msg import AckermannCommand, MotorFeedback

V_EPS = 1e-3  # |v| 小于该值视为静止（阿克曼不能原地自旋）


def twist_to_ackermann(v, w, wheel_radius, wheelbase, track_width,
                       max_steering_angle):
    """(线速度 v, 角速度 w) → ([左后, 右后] 轮角速度 rad/s, 转向角 δ rad)。

    自行车模型：δ = atan(w·L / v)，后轮速 = (v ∓ w_eff·track/2) / r，
    其中 w_eff 为转向角限幅后实际可达的角速度，保证 δ 被限幅时
    后轮差速与实际转弯半径一致。
    注意必须用 atan 而非 atan2：倒车（v<0）时 ω = v·tanδ/L 要求 δ
    与 v 异号，atan 的值域 ±π/2 正好给出正确符号，atan2 会把倒车
    转向角映射到 ±π/2 之外再被限幅，导致倒车转向反向。
    v≈0 时不能原地转向：后轮速为 0，δ 按 w 方向打满（预打方向）。
    """
    if abs(v) < V_EPS:
        if abs(w) < 1e-6:
            return [0.0, 0.0], 0.0
        steering = math.copysign(max_steering_angle, w)
        return [0.0, 0.0], steering
    steering = math.atan(w * wheelbase / v)
    steering = max(-max_steering_angle, min(max_steering_angle, steering))
    w_eff = v * math.tan(steering) / wheelbase
    left = (v - w_eff * track_width / 2.0) / wheel_radius
    right = (v + w_eff * track_width / 2.0) / wheel_radius
    return [left, right], steering


def ackermann_to_twist(rear_speeds, steering, wheel_radius, wheelbase):
    """([左后, 右后] 轮角速度 rad/s, 转向角 δ rad) → (线速度 v, 角速度 w)。"""
    v = (rear_speeds[0] + rear_speeds[1]) / 2.0 * wheel_radius
    w = v * math.tan(steering) / wheelbase
    return v, w


class SimMotorBridgeNode(Node):
    def __init__(self):
        super().__init__('sim_motor_bridge_node')
        self.declare_parameter('wheel_radius', 0.076)
        self.declare_parameter('wheelbase', 0.31)
        self.declare_parameter('track_width', 0.32)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('sim_voltage', 24.0)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.sim_voltage = self.get_parameter('sim_voltage').value

        self.create_subscription(
            AckermannCommand, '/ackermann_cmd', self.cmd_cb_, 10)
        self.create_subscription(
            Odometry, '/ugv/wheel/odometry', self.odom_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/ugv/sim/cmd_vel', 10)
        self.pub_feedback = self.create_publisher(
            MotorFeedback, '/motor_feedback', 10)

    def cmd_cb_(self, msg):
        """阿克曼指令 → 整车 (v, w) → 下发 Gazebo。"""
        v, w = ackermann_to_twist(
            list(msg.rear_speeds), msg.steering_angle,
            self.wheel_radius, self.wheelbase)
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)

    def odom_cb(self, msg):
        """gz 里程计 (v, w) → 反算后轮速 + 转向角 → 电机反馈（随里程计频率）。"""
        speeds, steering = twist_to_ackermann(
            msg.twist.twist.linear.x, msg.twist.twist.angular.z,
            self.wheel_radius, self.wheelbase, self.track_width,
            self.max_steering_angle)
        feedback = MotorFeedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.rear_speeds = [float(s) for s in speeds]
        feedback.steering_angle = float(steering)
        feedback.voltage = float(self.sim_voltage)
        self.pub_feedback.publish(feedback)


def main(args=None):
    rclpy.init(args=args)
    node = SimMotorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
