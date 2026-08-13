#!/usr/bin/env python3
"""R680 4WD 键盘遥控节点。

从键盘读取按键，向 cmd_vel 话题发布 geometry_msgs/Twist 速度指令，
适用于 Gazebo 仿真中的滑移转向小车，也可用于实车驱动节点调试。

按键说明：
    w / s : 前进 / 后退
    a / d : 左转 / 右转
    空格  : 立即停车
    q / z : 线速度档位 +10% / -10%
    e / c : 角速度档位 +10% / -10%
    Ctrl-C: 退出

按键松开超过 key_timeout（默认 0.5 s）后自动停车，
避免终端焦点丢失导致小车失控。

用法：
    ros2 run car_control teleop_keyboard
    ros2 run car_control teleop_keyboard --ros-args \
        -p cmd_vel_topic:=/car/cmd_vel -p linear_speed:=0.8
"""

import sys
import threading

import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

HELP = """
R680 4WD 键盘遥控
-----------------
  w : 前进      s : 后退
  a : 左转      d : 右转
  空格 : 立即停车

  q / z : 线速度档位 +10% / -10%
  e / c : 角速度档位 +10% / -10%

  Ctrl-C : 退出
"""

# 按键 -> (线速度方向, 角速度方向)
MOVE_BINDINGS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
}

# 按键 -> (线速度倍率, 角速度倍率)
SPEED_BINDINGS = {
    'q': (1.1, 1.0),
    'z': (1.0 / 1.1, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 1.0 / 1.1),
}

# 速度档位限幅
MAX_LINEAR = 2.0    # m/s
MIN_LINEAR = 0.05   # m/s
MAX_ANGULAR = 4.0   # rad/s
MIN_ANGULAR = 0.1   # rad/s


class TeleopKeyboard(Node):
    """键盘遥控节点：键盘线程读按键，定时器线程发 Twist。"""

    def __init__(self):
        super().__init__('teleop_keyboard')

        self.declare_parameter('cmd_vel_topic', '/car/cmd_vel')
        self.declare_parameter('linear_speed', 0.5)    # m/s，初始线速度档位
        self.declare_parameter('angular_speed', 1.0)   # rad/s，初始角速度档位
        self.declare_parameter('repeat_rate', 20.0)    # Hz，指令发布频率
        self.declare_parameter('key_timeout', 0.5)     # s，按键超时自动停车

        topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.key_timeout = self.get_parameter('key_timeout').value
        repeat_rate = self.get_parameter('repeat_rate').value

        self.publisher = self.create_publisher(Twist, topic, 10)

        self._target_linear = 0.0
        self._target_angular = 0.0
        self._last_key_time = self.get_clock().now()
        self._lock = threading.Lock()

        self.create_timer(1.0 / repeat_rate, self._publish_cmd)

        self.get_logger().info(f'键盘遥控已启动，目标话题：{topic}')
        self.get_logger().info(
            f'当前档位：线速度 {self.linear_speed:.2f} m/s，'
            f'角速度 {self.angular_speed:.2f} rad/s')
        print(HELP)

    def handle_key(self, key):
        """处理单个按键，更新目标速度或速度档位。"""
        with self._lock:
            if key in MOVE_BINDINGS:
                linear_dir, angular_dir = MOVE_BINDINGS[key]
                self._target_linear = linear_dir * self.linear_speed
                self._target_angular = angular_dir * self.angular_speed
                self._last_key_time = self.get_clock().now()
            elif key == ' ':
                self._target_linear = 0.0
                self._target_angular = 0.0
                self._last_key_time = self.get_clock().now()
                self.get_logger().info('急停')
            elif key in SPEED_BINDINGS:
                linear_scale, angular_scale = SPEED_BINDINGS[key]
                self.linear_speed = min(
                    MAX_LINEAR, max(MIN_LINEAR,
                                    self.linear_speed * linear_scale))
                self.angular_speed = min(
                    MAX_ANGULAR, max(MIN_ANGULAR,
                                     self.angular_speed * angular_scale))
                self.get_logger().info(
                    f'档位调整：线速度 {self.linear_speed:.2f} m/s，'
                    f'角速度 {self.angular_speed:.2f} rad/s')

    def stop(self):
        """停车并发送一次零速度指令。"""
        with self._lock:
            self._target_linear = 0.0
            self._target_angular = 0.0
        self.publisher.publish(Twist())

    def _publish_cmd(self):
        """定时发布速度指令；按键超时后自动停车。"""
        with self._lock:
            elapsed = (self.get_clock().now() - self._last_key_time).nanoseconds / 1e9
            if elapsed > self.key_timeout:
                self._target_linear = 0.0
                self._target_angular = 0.0

            cmd = Twist()
            cmd.linear.x = self._target_linear
            cmd.angular.z = self._target_angular

        self.publisher.publish(cmd)


def _read_keys(node):
    """键盘读取循环（独立线程，需要交互式终端）。"""
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    try:
        while rclpy.ok():
            key = sys.stdin.read(1)
            if key == '\x03':  # Ctrl-C
                break
            node.handle_key(key)
    except termios.error:
        node.get_logger().error('无法读取键盘：请在交互式终端中运行本节点')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    key_thread = threading.Thread(target=_read_keys, args=(node,), daemon=True)
    key_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
