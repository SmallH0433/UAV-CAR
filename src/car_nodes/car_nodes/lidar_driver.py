"""激光雷达驱动节点

发布：
  /scan (sensor_msgs/LaserScan)  frame_id=laser_frame
参数：
  port      (str,  '/dev/ttyUSB0')  串口设备
  baudrate  (int,  115200)          波特率
  simulate  (bool, True)            True=发布模拟场景
  range_min (float, 0.12)           最小量程 m
  range_max (float, 8.0)            最大量程 m
  scan_rate (int,  10)              扫描频率 Hz
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarDriverNode(Node):
    def __init__(self):
        super().__init__('lidar_driver_node')
        # 声明参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('simulate', True)
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('scan_rate', 10)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.simulate = self.get_parameter('simulate').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        scan_rate = self.get_parameter('scan_rate').value

        # 固定角分辨率：1° 一束，共 360 束
        self.num_beams = 360
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = 2.0 * math.pi / self.num_beams

        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)

        self.serial = None
        self._serial_warned = False
        if not self.simulate:
            try:
                import serial  # pyserial
                self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
                self.get_logger().info(f'已打开雷达串口 {self.port}')
            except Exception as e:
                self.get_logger().error(f'打开串口 {self.port} 失败：{e}，将发布空扫描')
        else:
            self.get_logger().info('仿真模式：正前方 3m 有墙的模拟场景')

        self.timer = self.create_timer(1.0 / scan_rate, self.timer_cb)

    def timer_cb(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.get_parameter('scan_rate').value
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        if self.simulate:
            msg.ranges = self._make_sim_ranges()
        else:
            msg.ranges = self._read_ranges()
        self.pub_scan.publish(msg)

    def _make_sim_ranges(self):
        """模拟场景：正前方 ±30° 返回 3m（一堵墙），其余返回最大量程"""
        ranges = []
        for i in range(self.num_beams):
            angle = self.angle_min + i * self.angle_increment
            if abs(angle) <= math.radians(30.0):
                ranges.append(3.0)
            else:
                ranges.append(self.range_max)
        return ranges

    def _read_ranges(self):
        """从串口读取一帧扫描并解析

        TODO: 按实际雷达协议（如思岚/镭神等）实现帧头查找、校验与距离解算。
        未实现时打印警告并返回空扫描，保证节点不崩溃。
        """
        if self.serial is None or not self.serial.is_open:
            return []
        try:
            data = self.serial.read(4096)
            if not data:
                return []
        except Exception as e:
            self.get_logger().warn(f'串口读取异常：{e}', throttle_duration_sec=2.0)
            return []
        # 协议解析未实现
        if not self._serial_warned:
            self.get_logger().warn('串口协议解析未实现（TODO），发布空扫描')
            self._serial_warned = True
        return []


def main(args=None):
    rclpy.init(args=args)
    node = LidarDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial is not None and node.serial.is_open:
            node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
