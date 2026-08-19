"""电机驱动节点（WHEELTEC R680 阿克曼底盘，STM32 下位机串口通信）

订阅：
  /ackermann_cmd (car_interfaces/AckermannCommand)  后轮速 + 转向角
发布：
  /motor_feedback (car_interfaces/MotorFeedback, 10Hz)
  /imu/data       (sensor_msgs/Imu, 随上行帧，publish_imu=true 时)
参数：
  port         (str,  '/dev/ttyACM0') 串口设备（控制板串口 3 的 USB 口，
                                        CH9102 在 Linux 上一般为 ttyACM*）
  baudrate     (int,  115200)         波特率（协议固定 115200）
  simulate     (bool, True)           True=反馈=指令值（不开串口）
  wheel_radius (float, 0.076)         轮半径 m（轮速↔车体速度换算）
  wheelbase    (float, 0.31)          轴距 m
  track_width  (float, 0.32)          后轮距 m
  max_steering_angle (float, 0.5)     前轮最大转向角 rad
  publish_imu  (bool, True)           是否由上行帧发布板载 IMU
  imu_frame    (str,  'imu_link')     IMU 消息 frame_id

真机协议为 WHEELTEC 二进制帧（0x7B 帧头 / BCC 异或校验 / 0x7D 帧尾），
编解码见 car_nodes/wheeltec_protocol.py。STM32 侧按车体三轴速度收发，
转向角 → 舵机、后轮差速由固件内部完成；本节点只负责
阿克曼指令 ↔ (vx, vz) 的运动学换算（与 sim_motor_bridge 同一套函数）。
注意固件由 vz 解算舵机转角时按前进假设处理（不随 vx<0 自动反向），
倒车时必须在下发前把 vz 翻号（见 ackermann_to_firmware_velocity），
否则前进/倒车切换时舵机角度不变，车会沿同一弧线往复卡死。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from car_interfaces.msg import AckermannCommand, MotorFeedback
from car_nodes.sim_motor_bridge import (
    ackermann_to_twist, twist_to_ackermann, V_EPS)
from car_nodes.wheeltec_protocol import (
    UplinkFrameParser, build_downlink_frame)


def ackermann_to_firmware_velocity(rear_speeds, steering, wheel_radius,
                                   wheelbase):
    """阿克曼指令（后轮速 + 转向角）→ 下位机下行帧的 (vx, vz)。

    运动学换算用 ackermann_to_twist（v 带符号，倒车 v<0）。但固件由 vz
    解算舵机转角时按前进假设处理（δ 只随 vz 符号变化，不随 vx<0 自动
    反向），因此倒车（vx<0）时把 vz 翻号，舵机才会打出与前进相反的
    角度，得到正确的倒车弧线（车头朝 ω 期望的方向甩）；
    否则前进/倒车共用同一舵机角，车沿同一弧线前后往复。
    v≈0 时 vz 强制为 0（阿克曼不能原地自旋）。
    上行反馈不受影响：上行 vx/vz 是车体实际运动，仍按标准 atan 换算。
    """
    vx, vz = ackermann_to_twist(rear_speeds, steering, wheel_radius, wheelbase)
    if abs(vx) < V_EPS:
        vz = 0.0
    elif vx < 0.0:
        vz = -vz
    return vx, vz


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        # 声明参数
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('simulate', True)
        self.declare_parameter('wheel_radius', 0.076)
        self.declare_parameter('wheelbase', 0.31)
        self.declare_parameter('track_width', 0.32)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('imu_frame', 'imu_link')

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.simulate = self.get_parameter('simulate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.publish_imu = self.get_parameter('publish_imu').value
        self.imu_frame = self.get_parameter('imu_frame').value

        # 目标指令：后轮速 [左后, 右后] rad/s + 转向角 rad
        self.target_rear = [0.0, 0.0]
        self.target_steering = 0.0
        self.last_feedback_rear = [0.0, 0.0]
        self.last_feedback_steering = 0.0
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
            AckermannCommand, '/ackermann_cmd', self.cmd_cb, 10)
        self.pub_feedback = self.create_publisher(
            MotorFeedback, '/motor_feedback', 10)
        self.pub_imu = None
        if self.publish_imu and not self.simulate:
            self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)

        # 10Hz 反馈
        self.timer = self.create_timer(0.1, self.feedback_timer_cb)

    def cmd_cb(self, msg):
        self.target_rear = list(msg.rear_speeds)[:2]
        self.target_steering = float(msg.steering_angle)
        if not self.simulate:
            self._send_command()

    def _send_command(self):
        """阿克曼指令 → 车体 (vx, vz) → WHEELTEC 下行帧写入 STM32。

        Y 轴速度恒为 0；Z 轴正值为逆时针（与 ROS 一致）。静止（v≈0）时
        vz 必须为 0——阿克曼底盘不能原地自旋；倒车时 vz 翻号以适配固件
        的前进假设（见 ackermann_to_firmware_velocity）。
        """
        if self.serial is None or not self.serial.is_open:
            return
        vx, vz = ackermann_to_firmware_velocity(
            self.target_rear, self.target_steering,
            self.wheel_radius, self.wheelbase)
        try:
            self.serial.write(build_downlink_frame(vx, 0.0, vz))
        except Exception as e:
            self.get_logger().warn(f'串口写入失败：{e}', throttle_duration_sec=2.0)

    def _read_feedback(self):
        """读取并解析上行帧，返回 (后轮速, 转向角, 电压)；无新帧时返回上次值"""
        if self.serial is None or not self.serial.is_open:
            return (self.last_feedback_rear, self.last_feedback_steering,
                    self.last_voltage)
        try:
            data = self.serial.read(256)
        except Exception as e:
            self.get_logger().warn(f'串口读取异常：{e}', throttle_duration_sec=2.0)
            return (self.last_feedback_rear, self.last_feedback_steering,
                    self.last_voltage)

        frames = self.parser.feed(data)
        if not frames:
            return (self.last_feedback_rear, self.last_feedback_steering,
                    self.last_voltage)

        latest = frames[-1]
        speeds, steering = twist_to_ackermann(
            latest['vx'], latest['vz'], self.wheel_radius, self.wheelbase,
            self.track_width, self.max_steering_angle)
        self.last_feedback_rear = speeds
        self.last_feedback_steering = steering
        self.last_voltage = latest['voltage']
        if latest['flag_stop'] != 0:
            self.get_logger().warn(
                '下位机报告电机失能（flag_stop≠0），请检查电机使能开关',
                throttle_duration_sec=5.0)
        if self.pub_imu is not None:
            self._publish_imu(latest)
        return speeds, steering, latest['voltage']

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
            speeds = list(self.target_rear)
            steering = self.target_steering
            # 假电压缓降，下限 10.5V
            self.fake_voltage = max(10.5, self.fake_voltage - 0.0001)
            voltage = self.fake_voltage
        else:
            speeds, steering, voltage = self._read_feedback()

        msg = MotorFeedback()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.rear_speeds = [float(s) for s in speeds[:2]]
        msg.steering_angle = float(steering)
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
