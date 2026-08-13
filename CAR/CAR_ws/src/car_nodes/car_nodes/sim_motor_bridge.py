"""仿真电机桥：替代实机 motor_driver，把轮速指令转成 Gazebo 差速指令。

订阅：
  /wheel_speeds       (car_interfaces/WheelSpeeds)  四轮目标角速度 rad/s
  /ugv/wheel/odometry (nav_msgs/Odometry)           gz 桥回的差速里程计
发布：
  /ugv/sim/cmd_vel    (geometry_msgs/Twist)         经 ros_gz_bridge 到 gz
  /motor_feedback     (car_interfaces/MotorFeedback) 由 gz 里程计反算轮速
参数：
  wheel_radius (float, 0.076)  轮半径 m
  track_width  (float, 0.32)   轮距 m
  sim_voltage  (float, 24.0)   上报的仿真电池电压 V

话题契约与实机 motor_driver 完全一致：chassis_controller 的里程计走
/motor_feedback 真实反馈路径，换到实机时只需用 motor_driver 替换本节点。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from car_interfaces.msg import MotorFeedback, WheelSpeeds


def wheel_speeds_to_twist(speeds, wheel_radius, track_width):
    """四轮角速度 [左前 右前 左后 右后] rad/s → (线速度 v, 角速度 w)。"""
    left = (speeds[0] + speeds[2]) / 2.0 * wheel_radius
    right = (speeds[1] + speeds[3]) / 2.0 * wheel_radius
    v = (left + right) / 2.0
    w = (right - left) / track_width
    return v, w


def twist_to_wheel_speeds(v, w, wheel_radius, track_width):
    """(线速度 v, 角速度 w) → 四轮角速度 [左前 右前 左后 右后] rad/s。"""
    left = (v - w * track_width / 2.0) / wheel_radius
    right = (v + w * track_width / 2.0) / wheel_radius
    return [left, right, left, right]


class SimMotorBridgeNode(Node):
    def __init__(self):
        super().__init__('sim_motor_bridge_node')
        self.declare_parameter('wheel_radius', 0.076)
        self.declare_parameter('track_width', 0.32)
        self.declare_parameter('sim_voltage', 24.0)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        self.sim_voltage = self.get_parameter('sim_voltage').value

        self.create_subscription(
            WheelSpeeds, '/wheel_speeds', self.wheels_cb, 10)
        self.create_subscription(
            Odometry, '/ugv/wheel/odometry', self.odom_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, '/ugv/sim/cmd_vel', 10)
        self.pub_feedback = self.create_publisher(
            MotorFeedback, '/motor_feedback', 10)

    def wheels_cb(self, msg):
        """四轮角速度 → 左右平均换算 (v, w) → 下发 Gazebo。"""
        v, w = wheel_speeds_to_twist(
            list(msg.speeds), self.wheel_radius, self.track_width)
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)

    def odom_cb(self, msg):
        """gz 里程计 (v, w) → 反算四轮角速度 → 电机反馈（10~30Hz 随里程计）。"""
        speeds = twist_to_wheel_speeds(
            msg.twist.twist.linear.x, msg.twist.twist.angular.z,
            self.wheel_radius, self.track_width)
        feedback = MotorFeedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.speeds = [float(s) for s in speeds]
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
