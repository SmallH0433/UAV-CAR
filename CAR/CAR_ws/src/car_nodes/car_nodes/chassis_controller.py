"""底盘控制器节点（4WD 滑移转向）

订阅：
  /cmd_vel        (geometry_msgs/Twist)
  /motor_feedback (car_interfaces/MotorFeedback)
发布：
  /wheel_speeds (car_interfaces/WheelSpeeds)
  /odom         (nav_msgs/Odometry)
  tf: odom -> base_footprint
服务：
  /chassis/emergency_stop (std_srvs/SetBool)  true=急停锁存 false=解除
参数：
  wheel_radius    (float, 0.076)  轮半径 m
  track_width     (float, 0.32)   轮距 m
  cmd_timeout     (float, 0.5)    cmd_vel 超时自动停车 s
  max_wheel_speed (float, 15.0)   轮角速度限幅 rad/s
  odom_frame      (str, 'odom')
  base_frame      (str, 'base_footprint')
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

from car_interfaces.msg import MotorFeedback, WheelSpeeds


class ChassisControllerNode(Node):
    def __init__(self):
        super().__init__('chassis_controller_node')
        # 声明参数
        self.declare_parameter('wheel_radius', 0.076)
        self.declare_parameter('track_width', 0.32)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('max_wheel_speed', 15.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # 状态
        self.last_cmd = Twist()
        self.last_cmd_time = None
        self.feedback_speeds = None      # 实际轮速（有反馈时用于里程计）
        self.emergency_stopped = False
        # 里程计积分状态
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = self.get_clock().now()

        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(
            MotorFeedback, '/motor_feedback', self.feedback_cb, 10)
        self.pub_wheels = self.create_publisher(WheelSpeeds, '/wheel_speeds', 10)
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
        self.feedback_speeds = list(msg.speeds)

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

        # 逆运动学：差速 → 四轮角速度（左前=左后，右前=右后）
        v_left = v - w * self.track_width / 2.0
        v_right = v + w * self.track_width / 2.0
        w_left = self._clamp_wheel(v_left / self.wheel_radius)
        w_right = self._clamp_wheel(v_right / self.wheel_radius)

        ws = WheelSpeeds()
        # 顺序：左前、右前、左后、右后
        ws.speeds = [w_left, w_right, w_left, w_right]
        self.pub_wheels.publish(ws)

        # 里程计：优先用电机反馈，无反馈时用当前指令值
        if self.feedback_speeds is not None:
            s = self.feedback_speeds
            odom_v = (s[0] + s[2]) / 2.0 * self.wheel_radius  # 左侧平均
            odom_v += (s[1] + s[3]) / 2.0 * self.wheel_radius  # 右侧平均
            odom_v /= 2.0
            left_v = (s[0] + s[2]) / 2.0 * self.wheel_radius
            right_v = (s[1] + s[3]) / 2.0 * self.wheel_radius
            odom_w = (right_v - left_v) / self.track_width
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
