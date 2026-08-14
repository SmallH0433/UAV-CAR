"""底盘控制器节点（阿克曼转向：前轮舵机转向 + 后轮双电机驱动）

订阅：
  /cmd_vel        (geometry_msgs/Twist)
  /motor_feedback (car_interfaces/MotorFeedback)
发布：
  /ackermann_cmd (car_interfaces/AckermannCommand)
  /odom         (nav_msgs/Odometry)
  tf: odom -> base_footprint
服务：
  /chassis/emergency_stop (std_srvs/SetBool)  true=急停锁存 false=解除
参数：
  wheel_radius       (float, 0.076) 轮半径 m
  track_width        (float, 0.32)  后轮距 m
  wheelbase          (float, 0.31)  轴距 m（实机联调时实测修正）
  max_steering_angle (float, 0.5)   前轮最大转向角 rad（联调校准）
  cmd_timeout        (float, 0.5)   cmd_vel 超时自动停车 s
  max_wheel_speed    (float, 15.0)  轮角速度限幅 rad/s
  odom_frame         (str, 'odom')
  base_frame         (str, 'base_footprint')

运动学为自行车模型：转向角 δ = atan2(ω·L, v)，后轮速 v ± ω·track/2。
注意阿克曼底盘不能原地自旋：|v| ≈ 0 时后轮速强制为 0，转向角按 ω
方向打满（预打方向，车一动即转弯）。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

from car_interfaces.msg import AckermannCommand, MotorFeedback
from car_nodes.sim_motor_bridge import twist_to_ackermann, ackermann_to_twist


class ChassisControllerNode(Node):
    def __init__(self):
        super().__init__('chassis_controller_node')
        # 声明参数
        self.declare_parameter('wheel_radius', 0.076)
        self.declare_parameter('track_width', 0.32)
        self.declare_parameter('wheelbase', 0.31)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('max_wheel_speed', 15.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # 状态
        self.last_cmd = Twist()
        self.last_cmd_time = None
        self.feedback_rear = None        # 实际后轮速 [左后, 右后] rad/s
        self.feedback_steering = 0.0     # 实际转向角 rad
        self.emergency_stopped = False
        # 里程计积分状态
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = self.get_clock().now()

        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(
            MotorFeedback, '/motor_feedback', self.feedback_cb, 10)
        self.pub_cmd = self.create_publisher(
            AckermannCommand, '/ackermann_cmd', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_service(
            SetBool, '/chassis/emergency_stop', self.emergency_cb)

        # 50Hz 控制/里程计循环
        self.timer = self.create_timer(0.02, self.control_loop)

    def cmd_cb(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def feedback_cb(self, msg):
        self.feedback_rear = list(msg.rear_speeds)
        self.feedback_steering = float(msg.steering_angle)

    def emergency_cb(self, request, response):
        self.emergency_stopped = bool(request.data)
        response.success = True
        response.message = '已急停' if self.emergency_stopped else '已解除急停'
        self.get_logger().warn(response.message)
        return response

    def control_loop(self):
        now = self.get_clock().now()

        # 决定当前生效的 cmd_vel
        if self.emergency_stopped:
            v, w = 0.0, 0.0
        else:
            cmd = self.last_cmd
            if self.last_cmd_time is not None:
                dt_cmd = (now - self.last_cmd_time).nanoseconds * 1e-9
                if dt_cmd > self.cmd_timeout:
                    cmd = Twist()  # 超时自动停车
            else:
                cmd = Twist()
            v, w = cmd.linear.x, cmd.angular.z

        # 逆运动学：自行车模型 → 后轮速 + 转向角（含 v≈0 保护）
        rear_speeds, steering = twist_to_ackermann(
            v, w, self.wheel_radius, self.wheelbase, self.track_width,
            self.max_steering_angle)
        rear_speeds = [self._clamp_wheel(s) for s in rear_speeds]

        cmd_msg = AckermannCommand()
        cmd_msg.rear_speeds = rear_speeds
        cmd_msg.steering_angle = steering
        self.pub_cmd.publish(cmd_msg)

        # 里程计：优先用电机反馈，无反馈时用当前指令值
        if self.feedback_rear is not None:
            odom_v, odom_w = ackermann_to_twist(
                self.feedback_rear, self.feedback_steering,
                self.wheel_radius, self.wheelbase)
        else:
            odom_v, odom_w = v, w

        # 积分位姿
        dt = (now - self.last_odom_time).nanoseconds * 1e-9
        self.last_odom_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 0.0  # 时钟跳变保护
        self.x += odom_v * math.cos(self.yaw) * dt
        self.y += odom_v * math.sin(self.yaw) * dt
        self.yaw += odom_w * dt

        self._publish_odom(now, odom_v, odom_w)

    def _publish_odom(self, now, v, w):
        stamp = now.to_msg()
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.pub_odom.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    def _clamp_wheel(self, w):
        return max(-self.max_wheel_speed, min(self.max_wheel_speed, w))


def main(args=None):
    rclpy.init(args=args)
    node = ChassisControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
