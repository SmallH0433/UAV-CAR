"""无人机桥接节点

对外（UDP，JSON 文本协议）：
  收 {"cmd_type":..., "x":..., "y":..., "yaw":..., "vx":..., "wz":..., "mode":...}
     cmd_type=1 → 发布 /goal_pose；其余 → 发布 /uav/command 记录指令
  发 周期性把 UavStatus 序列化为 JSON 发到 uav_ip:uav_port
对内：
  订阅：/odom (nav_msgs/Odometry)、/perception/obstacles (car_interfaces/ObstacleArray)
  发布：/goal_pose (geometry_msgs/PoseStamped)
        /uav/command (car_interfaces/UavCommand)
        /uav/status (car_interfaces/UavStatus)
服务：
  /uav_bridge/set_mode (std_srvs/SetBool)  true=协同模式(2) false=自主模式(1)
参数：
  uav_ip           (str,   '192.168.1.100')
  uav_port         (int,   8888)
  listen_port      (int,   8889)
  enable           (bool,  True)   False 时不开 socket，只做内部桥接
  status_rate      (float, 2.0)    状态上报频率 Hz
  allow_direct_vel (bool,  False)  True 时 cmd_type=2 的 velocity 直发 /cmd_vel（绕过避障）
"""

import json
import math
import socket

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

from car_interfaces.msg import ObstacleArray, UavCommand, UavStatus


class UavBridgeNode(Node):
    def __init__(self):
        super().__init__('uav_bridge_node')
        # 声明参数
        self.declare_parameter('uav_ip', '192.168.1.100')
        self.declare_parameter('uav_port', 8888)
        self.declare_parameter('listen_port', 8889)
        self.declare_parameter('enable', True)
        self.declare_parameter('status_rate', 2.0)
        self.declare_parameter('allow_direct_vel', False)

        self.uav_ip = self.get_parameter('uav_ip').value
        self.uav_port = self.get_parameter('uav_port').value
        listen_port = self.get_parameter('listen_port').value
        self.enable = self.get_parameter('enable').value
        status_rate = self.get_parameter('status_rate').value
        self.allow_direct_vel = self.get_parameter('allow_direct_vel').value

        # 内部状态
        self.mode = 0               # 0=待机 1=自主避障 2=无人机协同
        self.odom = None
        self.obstacles = []

        # 对内接口
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(
            ObstacleArray, '/perception/obstacles', self.obstacles_cb, 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_uav_cmd = self.create_publisher(UavCommand, '/uav/command', 10)
        self.pub_status = self.create_publisher(UavStatus, '/uav/status', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_service(SetBool, '/uav_bridge/set_mode', self.set_mode_cb)

        # 对外 UDP socket
        self.sock = None
        if self.enable:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sock.bind(('0.0.0.0', listen_port))
                self.sock.setblocking(False)
                self.get_logger().info(f'UDP 监听端口 {listen_port}，'
                                       f'上报目标 {self.uav_ip}:{self.uav_port}')
            except OSError as e:
                self.get_logger().error(f'UDP socket 初始化失败：{e}')
                self.sock = None
        else:
            self.get_logger().info('enable=False，仅做内部话题桥接，不开 socket')

        self.recv_timer = self.create_timer(0.02, self.recv_timer_cb)
        self.status_timer = self.create_timer(1.0 / status_rate, self.status_timer_cb)

    # ---------- 对内订阅 ----------
    def odom_cb(self, msg):
        self.odom = msg

    def obstacles_cb(self, msg):
        self.obstacles = list(msg.obstacles)

    def set_mode_cb(self, request, response):
        self.mode = 2 if request.data else 1
        response.success = True
        response.message = '已切换到无人机协同模式' if request.data else '已切换到自主模式'
        self.get_logger().info(response.message)
        return response

    # ---------- UDP 接收 ----------
    def recv_timer_cb(self):
        if self.sock is None:
            return
        while True:
            try:
                data, _addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as e:
                self.get_logger().warn(f'UDP 接收异常：{e}', throttle_duration_sec=2.0)
                break
            try:
                cmd = json.loads(data.decode('utf-8'))
            except (ValueError, UnicodeDecodeError):
                self.get_logger().warn('收到非法 JSON 指令，已丢弃',
                                       throttle_duration_sec=2.0)
                continue
            self._handle_uav_command(cmd)

    def _handle_uav_command(self, cmd):
        cmd_type = int(cmd.get('cmd_type', 0))
        if cmd_type == 1:
            # 前往目标点：直接发布 /goal_pose（走避障）
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'odom'
            goal.pose.position.x = float(cmd.get('x', 0.0))
            goal.pose.position.y = float(cmd.get('y', 0.0))
            yaw = float(cmd.get('yaw', 0.0))
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.orientation.w = math.cos(yaw / 2.0)
            self.pub_goal.publish(goal)
        # 所有指令都通过 /uav/command 记录
        uav_cmd = UavCommand()
        uav_cmd.header.stamp = self.get_clock().now().to_msg()
        uav_cmd.cmd_type = cmd_type
        uav_cmd.target_pose.position.x = float(cmd.get('x', 0.0))
        uav_cmd.target_pose.position.y = float(cmd.get('y', 0.0))
        uav_cmd.velocity.linear.x = float(cmd.get('vx', 0.0))
        uav_cmd.velocity.angular.z = float(cmd.get('wz', 0.0))
        uav_cmd.mode = int(cmd.get('mode', 0))
        self.pub_uav_cmd.publish(uav_cmd)

        if cmd_type == 3:
            self.mode = uav_cmd.mode
        elif cmd_type == 2 and self.allow_direct_vel:
            # 速度直控：绕过避障，默认关闭
            twist = Twist()
            twist.linear.x = uav_cmd.velocity.linear.x
            twist.angular.z = uav_cmd.velocity.angular.z
            self.pub_cmd_vel.publish(twist)

    # ---------- 状态上报 ----------
    def status_timer_cb(self):
        status = UavStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        if self.odom is not None:
            status.pose = self.odom.pose.pose
            status.speed = float(self.odom.twist.twist.linear.x)
        status.mode = self.mode
        if self.obstacles:
            # 前方 ±45° 视为"前方"
            ahead = [o for o in self.obstacles if abs(o.angle) < math.pi / 4.0]
            status.obstacle_ahead = len(ahead) > 0
            status.min_obstacle_distance = float(
                min(o.distance for o in self.obstacles))
        else:
            status.obstacle_ahead = False
            status.min_obstacle_distance = 99.0  # 无障碍时给一个大的占位值
        self.pub_status.publish(status)

        if self.sock is not None:
            payload = {
                'x': status.pose.position.x,
                'y': status.pose.position.y,
                'speed': status.speed,
                'mode': status.mode,
                'battery_voltage': status.battery_voltage,
                'obstacle_ahead': status.obstacle_ahead,
                'min_obstacle_distance': status.min_obstacle_distance,
            }
            try:
                self.sock.sendto(json.dumps(payload).encode('utf-8'),
                                 (self.uav_ip, self.uav_port))
            except OSError as e:
                self.get_logger().warn(f'UDP 发送失败：{e}', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = UavBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.sock is not None:
            node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
