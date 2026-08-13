"""电机驱动节点（WHEELTEC R680 底盘，STM32 下位机串口通信）

订阅：
  /wheel_speeds (car_interfaces/WheelSpeeds)  四轮目标角速度 rad/s
发布：
  /motor_feedback (car_interfaces/MotorFeedback, 10Hz)
  /imu/data       (sensor_msgs/Imu, 随上行帧，publish_imu=true 时)
参数：
  port         (str,  '/dev/ttyACM0') 串口设备（控制板串口 3 的 USB 口，
                                        CH9102 在 Linux 上一般为 ttyACM*）
  baudrate     (int,  115200)         波特率（协议固定 115200）
  simulate     (bool, True)           True=反馈=指令值（不开串口）
  num_motors   (int,  4)              电机数量
  wheel_radius (float, 0.076)         轮半径 m（轮速↔车体速度换算）
  track_width  (float, 0.32)          轮距 m
  publish_imu  (bool, True)           是否由上行帧发布板载 IMU
  imu_frame    (str,  'imu_link')     IMU 消息 frame_id

真机协议为 WHEELTEC 二进制帧（0x7B 帧头 / BCC 异或校验 / 0x7D 帧尾），
编解码见 car_nodes/wheeltec_protocol.py。STM32 侧按车体三轴速度收发，
本节点负责 四轮角速度 ↔ (vx, vz) 的运动学换算（与 sim_motor_bridge 一致）。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from car_interfaces.msg import MotorFeedback, WheelSpeeds
from car_nodes.sim_motor_bridge import (
    twist_to_wheel_speeds, wheel_speeds_to_twist)
from car_nodes.wheeltec_protocol import (
    UplinkFrameParser, build_downlink_frame)


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        # 声明参数
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('simulate', True)
        self.declare_parameter('num_motors', 4)
        self.declare_parameter('wheel_radius', 0.076)
        self.declare_parameter('track_width', 0.32)
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('imu_frame', 'imu_link')

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.simulate = self.get_parameter('simulate').value
        self.num_motors = self.get_parameter('num_motors').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        self.publish_imu = self.get_parameter('publish_imu').value
        self.imu_frame = self.get_parameter('imu_frame').value

        self.target_speeds = [0.0] * self.num_motors
        self.last_feedback = [0.0] * self.num_motors
        self.last_voltage = 0.0
        self.fake_voltage = 12.0  # 仿真电压（缓降）

        self.serial = None
        self.parser = UplinkFrameParser()
        if not self.simulate:
            try:
                import serial  # pyserial
                self.serial = serial.Serial(self.port, self.baudrate, timeout=0.05)
                self.get_logger().info(f'已打开电机串口 {self.port}')
            except Exception as e:
                self.get_logger().error(f'打开串口 {self.port} 失败：{e}')
        else:
            self.get_logger().info('仿真模式：反馈=指令值')

        self.create_subscription(
            WheelSpeeds, '/wheel_speeds', self.wheels_cb, 10)
        self.pub_feedback = self.create_publisher(
            MotorFeedback, '/motor_feedback', 10)
        self.pub_imu = None
        if self.publish_imu and not self.simulate:
            self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)

        # 10Hz 反馈
        self.timer = self.create_timer(0.1, self.feedback_timer_cb)

    def wheels_cb(self, msg):
        self.target_speeds = list(msg.speeds)[:self.num_motors]
        if not self.simulate:
            self._send_command()

    def _send_command(self):
        """四轮角速度 → 车体 (vx, vz) → WHEELTEC 下行帧写入 STM32。

        差速车型 Y 轴速度恒为 0；Z 轴正值为逆时针（与 ROS 一致）。
        """
        if self.serial is None or not self.serial.is_open:
            return
        vx, vz = wheel_speeds_to_twist(
            self.target_speeds, self.wheel_radius, self.track_width)
        try:
            self.serial.write(build_downlink_frame(vx, 0.0, vz))
        except Exception as e:
            self.get_logger().warn(f'串口写入失败：{e}', throttle_duration_sec=2.0)

    def _read_feedback(self):
        """读取并解析上行帧，返回 (四轮角速度, 电压)；无新帧时返回上次值"""
        if self.serial is None or not self.serial.is_open:
            return self.last_feedback, self.last_voltage
        try:
            data = self.serial.read(256)
        except Exception as e:
            self.get_logger().warn(f'串口读取异常：{e}', throttle_duration_sec=2.0)
            return self.last_feedback, self.last_voltage

        frames = self.parser.feed(data)
        if not frames:
            return self.last_feedback, self.last_voltage

        latest = frames[-1]
        speeds = twist_to_wheel_speeds(
            latest['vx'], latest['vz'], self.wheel_radius, self.track_width)
        self.last_feedback = speeds
        self.last_voltage = latest['voltage']
        if latest['flag_stop'] != 0:
            self.get_logger().warn(
                '下位机报告电机失能（flag_stop≠0），请检查电机使能开关',
                throttle_duration_sec=5.0)
        if self.pub_imu is not None:
            self._publish_imu(latest)
        return speeds, latest['voltage']

    def _publish_imu(self, frame):
        """上行帧中的板载 IMU 原始数据 → sensor_msgs/Imu。

        加速度 /1672 → m/s²，角速度 /3753 → rad/s（厂商给定量程）。
        下位机不提供姿态角，orientation 标记为不可用。
        """
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        msg.linear_acceleration.x = frame['accel'][0]
        msg.linear_acceleration.y = frame['accel'][1]
        msg.linear_acceleration.z = frame['accel'][2]
        msg.angular_velocity.x = frame['gyro'][0]
        msg.angular_velocity.y = frame['gyro'][1]
        msg.angular_velocity.z = frame['gyro'][2]
        msg.orientation_covariance[0] = -1.0  # 无姿态角估计
        self.pub_imu.publish(msg)

    def feedback_timer_cb(self):
        if self.simulate:
            speeds = list(self.target_speeds)
            # 假电压缓降，下限 10.5V
            self.fake_voltage = max(10.5, self.fake_voltage - 0.0001)
            voltage = self.fake_voltage
        else:
            speeds, voltage = self._read_feedback()

        # MotorFeedback.speeds 为定长 float32[4]，不足补 0
        out = [float(s) for s in speeds[:4]]
        out += [0.0] * (4 - len(out))
        msg = MotorFeedback()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.speeds = out
        msg.voltage = float(voltage)
        self.pub_feedback.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
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
