"""电机驱动节点（4 电机 UART）

订阅：
  /wheel_speeds (car_interfaces/WheelSpeeds)
发布：
  /motor_feedback (car_interfaces/MotorFeedback, 10Hz)
参数：
  port       (str,  '/dev/ttyS1')  串口设备
  baudrate   (int,  115200)        波特率
  simulate   (bool, True)          True=反馈=指令值
  num_motors (int,  4)             电机数量
"""

import rclpy
from rclpy.node import Node

from car_interfaces.msg import MotorFeedback, WheelSpeeds


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        # 声明参数
        self.declare_parameter('port', '/dev/ttyS1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('simulate', True)
        self.declare_parameter('num_motors', 4)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.simulate = self.get_parameter('simulate').value
        self.num_motors = self.get_parameter('num_motors').value

        self.target_speeds = [0.0] * self.num_motors
        self.last_feedback = [0.0] * self.num_motors
        self.fake_voltage = 12.0  # 仿真电压（缓降）

        self.serial = None
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

        # 10Hz 反馈
        self.timer = self.create_timer(0.1, self.feedback_timer_cb)

    def wheels_cb(self, msg):
        self.target_speeds = list(msg.speeds)[:self.num_motors]
        if not self.simulate:
            self._send_command()

    def _send_command(self):
        """下发速度指令：简单文本协议 "V w0 w1 w2 w3\n"

        TODO: 按实际电调协议（帧头/校验/二进制格式等）修改。
        """
        if self.serial is None or not self.serial.is_open:
            return
        try:
            line = 'V ' + ' '.join(f'{w:.3f}' for w in self.target_speeds) + '\n'
            self.serial.write(line.encode('ascii'))
        except Exception as e:
            self.get_logger().warn(f'串口写入失败：{e}', throttle_duration_sec=2.0)

    def _read_feedback(self):
        """解析回读：期望 "F w0 w1 w2 w3 voltage\n"，超时/失败返回上次值"""
        if self.serial is None or not self.serial.is_open:
            return self.last_feedback, 0.0
        try:
            line = self.serial.readline().decode('ascii', errors='ignore').strip()
            parts = line.split()
            if len(parts) >= self.num_motors + 2 and parts[0] == 'F':
                speeds = [float(x) for x in parts[1:self.num_motors + 1]]
                voltage = float(parts[self.num_motors + 1])
                self.last_feedback = speeds
                return speeds, voltage
        except (ValueError, UnicodeDecodeError) as e:
            self.get_logger().warn(f'回读解析失败：{e}', throttle_duration_sec=2.0)
        except Exception as e:
            self.get_logger().warn(f'串口读取异常：{e}', throttle_duration_sec=2.0)
        return self.last_feedback, 0.0

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
