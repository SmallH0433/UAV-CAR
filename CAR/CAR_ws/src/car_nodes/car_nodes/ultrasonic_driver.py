"""HC-SR04 超声波测距驱动节点（GPIO 直驱，树莓派）

发布：
  /ultrasonic/range (sensor_msgs/Range)  车尾障碍物距离 m（仅发布有效测量；
      测量超时/失败不发布，订阅方按数据过期处理）
参数：
  trig_pin  (int, 14)           触发引脚 BCM 编号（实机接 TXD1/GPIO14）
  echo_pin  (int, 15)           回波引脚 BCM 编号（实机接 RXD1/GPIO15）
  gpio_chip (str, gpiochip0)    GPIO 芯片
  rate      (float, 10.0)       测量频率 Hz（HC-SR04 周期建议 ≥60ms，勿超 15）
  simulate  (bool, False)       true=无硬件自检，固定发布 max_range
  frame_id  (str, ultrasonic_link)

硬件注意：HC-SR04 为 5V 器件，Echo 输出 5V 电平，实机已在 Echo→GPIO15
之间加分压模块（勿直插 3.3V GPIO）。测量在独立线程进行（阻塞等回波
最长 25ms），不占用 rclpy 执行器。
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

try:
    import gpiod
    _GPIOD_AVAILABLE = True
except ImportError:
    gpiod = None
    _GPIOD_AVAILABLE = False


class UltrasonicDriver(Node):
    def __init__(self):
        super().__init__('ultrasonic_driver')
        self.declare_parameter('trig_pin', 14)
        self.declare_parameter('echo_pin', 15)
        self.declare_parameter('gpio_chip', 'gpiochip0')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('simulate', False)
        self.declare_parameter('frame_id', 'ultrasonic_link')

        self.trig_pin = int(self.get_parameter('trig_pin').value)
        self.echo_pin = int(self.get_parameter('echo_pin').value)
        self.gpio_chip = str(self.get_parameter('gpio_chip').value)
        self.rate = float(self.get_parameter('rate').value)
        self.simulate = bool(self.get_parameter('simulate').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.pub = self.create_publisher(Range, '/ultrasonic/range', 10)

        self._chip = None
        self._trig = None
        self._echo = None
        if not self.simulate:
            self._open_gpio()

        import threading
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._measure_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'超声波测距已启动（trig=GPIO{self.trig_pin} '
            f'echo=GPIO{self.echo_pin} simulate={self.simulate}）')

    def _open_gpio(self):
        if not _GPIOD_AVAILABLE:
            self.get_logger().error('gpiod 未安装（sudo apt install '
                                    'python3-libgpiod），无法读取超声波')
            return
        try:
            self._chip = gpiod.Chip(self.gpio_chip)
            self._trig = self._chip.get_line(self.trig_pin)
            self._echo = self._chip.get_line(self.echo_pin)
            self._trig.request(consumer='ultrasonic_driver',
                               type=gpiod.LINE_REQ_DIR_OUT, default_val=0)
            self._echo.request(consumer='ultrasonic_driver',
                               type=gpiod.LINE_REQ_DIR_IN)
        except OSError as exc:
            self.get_logger().error(
                f'GPIO 打开失败（trig={self.trig_pin} echo={self.echo_pin}）：'
                f'{exc}；将以无数据状态运行')
            self._chip = self._trig = self._echo = None

    def _measure_once(self):
        """触发一次测距，返回距离 m；超时/无回波返回 None。"""
        self._trig.set_value(1)
        time.sleep(0.000015)  # >10us 触发脉冲
        self._trig.set_value(0)
        deadline = time.monotonic() + 0.025
        while self._echo.get_value() == 0:
            if time.monotonic() > deadline:
                return None
        t_start = time.monotonic()
        deadline = t_start + 0.025
        while self._echo.get_value() == 1:
            if time.monotonic() > deadline:
                return None
        return (time.monotonic() - t_start) * 343.0 / 2.0

    def _measure_loop(self):
        period = max(0.065, 1.0 / max(self.rate, 0.5))
        while not self._stop.is_set():
            if self.simulate:
                self._publish(4.0)
            elif self._trig is not None:
                try:
                    distance = self._measure_once()
                except OSError as exc:
                    self.get_logger().warn(
                        f'GPIO 读取异常：{exc}', throttle_duration_sec=5.0)
                    distance = None
                if distance is not None:
                    # 限幅到传感器有效量程
                    self._publish(min(max(distance, 0.02), 4.0))
            self._stop.wait(period)

    def _publish(self, distance):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # HC-SR04 约 15°
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = float(distance)
        self.pub.publish(msg)

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
