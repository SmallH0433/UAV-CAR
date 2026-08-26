"""双向丝杆驱动节点（ESP8266 下位机：双 42 步进 + T8 双向丝杆）

订阅：
  /leadscrew/cmd    (car_interfaces/LeadscrewCommand)  控制组指令
发布：
  /leadscrew/status (car_interfaces/LeadscrewStatus, 2Hz + 状态变化时)
  /leadscrew/sim/pusher_{a,b,c,d}/cmd_pos (std_msgs/Float64, 10Hz,
    仅 simulate=true 且 publish_sim_joints=true：驱动 Gazebo 模型
    r680_4wd 的 4 个推杆棱柱关节；组1→pusher_a/pusher_c，
    组2→pusher_b/pusher_d。q=0 螺母在外侧(pos=0)，q=-0.057 到内侧
    (pos=travel_mm)，与 model.sdf 关节限位一致)
参数：
  port           (str,   '/dev/ttyUSB0') ESP8266 USB 串口
  baudrate       (int,   115200)         波特率（固件固定 115200）
  simulate       (bool,  True)           True=本地模拟状态机（不开串口）
  publish_sim_joints (bool, True)        仿真时同步发布 Gazebo 推杆关节指令
  status_period  (float, 2.0)            实机模式 POS 轮询周期 s
  travel_mm      (float, 57.0)           单边行程 mm（仿真用）
  sim_speed_mm_s (float, 2.0)            仿真螺母移动速度 mm/s

固件工程：tools/leadscrew42（仓库 esp8266-deploy 分支）。文本协议编解码
见 car_nodes/esp_leadscrew_protocol.py。每个控制组 = 电机 + 双向丝杆 +
左右两螺母，状态机 AT_OUTER/AT_INNER/HOLD_MID（自锁）与 MOVING_IN/OUT。
硬件注意：ESP8266 上电假定螺母在外侧（pos=0），实际不在时先手动归位；
USB 串口打开瞬间 DTR/RTS 会触发 ESP 自动复位，本节点打开后显式释放
并丢弃上电 banner；电机 12V，驱动板 24V 供电时电流档须按额定电流设。
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from car_interfaces.msg import LeadscrewCommand, LeadscrewStatus
from car_nodes.esp_leadscrew_protocol import (
    CMD_STOP, CMD_IN, CMD_OUT, CMD_RELAX, CMD_LOCK,
    STATE_NAMES, build_command, parse_line)

STATE_AT_OUTER, STATE_MOVING_IN, STATE_AT_INNER, STATE_MOVING_OUT, \
    STATE_HOLD_MID = range(5)

# 控制组 → Gazebo 推杆关节映射（r680_4wd/model.sdf）：
# 组1 = 电机A（A 边中点下方）→ pusher_a/pusher_c；组2 = 电机B → pusher_b/pusher_d
SIM_JOINT_TOPICS = {0: ('/leadscrew/sim/pusher_a/cmd_pos',
                        '/leadscrew/sim/pusher_c/cmd_pos'),
                    1: ('/leadscrew/sim/pusher_b/cmd_pos',
                        '/leadscrew/sim/pusher_d/cmd_pos')}


class LeadscrewDriverNode(Node):
    def __init__(self):
        super().__init__('leadscrew_driver_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('simulate', True)
        self.declare_parameter('publish_sim_joints', True)
        self.declare_parameter('status_period', 2.0)
        self.declare_parameter('travel_mm', 57.0)
        self.declare_parameter('sim_speed_mm_s', 2.0)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.simulate = self.get_parameter('simulate').value
        self.status_period = self.get_parameter('status_period').value
        self.travel_mm = self.get_parameter('travel_mm').value
        self.sim_speed = self.get_parameter('sim_speed_mm_s').value

        # 两组状态缓存（下标 0=组1 1=组2）；实机模式由 POS/DONE 回报刷新
        self.state = [STATE_AT_OUTER, STATE_AT_OUTER]
        self.pos_mm = [0.0, 0.0]
        self.enabled = [False, False]
        self.speed_steps = 1600

        self.serial = None
        self.rx_buf = ''
        self.last_pos_query = 0.0
        if not self.simulate:
            try:
                import serial  # pyserial
                self.serial = serial.Serial(self.port, self.baudrate,
                                            timeout=0.05)
                # 打开串口会拉 DTR/RTS 触发 ESP 自动复位；显式释放，
                # 等固件重启完成后丢弃 banner
                self.serial.dtr = False
                self.serial.rts = False
                time.sleep(1.0)
                self.serial.reset_input_buffer()
                self._write(b'POS\n')
                self.get_logger().info(f'已打开丝杆串口 {self.port}')
            except Exception as e:
                self.get_logger().error(f'打开串口 {self.port} 失败：{e}')
        else:
            self.get_logger().info('仿真模式：本地模拟状态机，不开串口')

        # 仿真模式同步驱动 Gazebo 推杆关节（publish_sim_joints=true 时）
        self.sim_joint_pubs = {}
        if self.simulate and self.get_parameter('publish_sim_joints').value:
            for grp, topics in SIM_JOINT_TOPICS.items():
                self.sim_joint_pubs[grp] = [
                    self.create_publisher(Float64, t, 10) for t in topics]
            self.get_logger().info('仿真推杆关节指令发布已启用（4 关节）')

        self.create_subscription(
            LeadscrewCommand, '/leadscrew/cmd', self.cmd_cb, 10)
        self.pub_status = self.create_publisher(
            LeadscrewStatus, '/leadscrew/status', 10)

        self.last_pub = 0.0
        self.timer = self.create_timer(0.1, self.timer_cb)

    # ---------- 下行 ----------
    def cmd_cb(self, msg):
        if msg.command not in (CMD_STOP, CMD_IN, CMD_OUT, CMD_RELAX, CMD_LOCK):
            self.get_logger().warn(f'未知 command={msg.command}，忽略')
            return
        if msg.speed:
            self.speed_steps = msg.speed
        if self.simulate:
            self._sim_command(msg.group, msg.command)
            return
        if self.serial is None or not self.serial.is_open:
            self.get_logger().warn('串口未打开，指令被丢弃',
                                   throttle_duration_sec=2.0)
            return
        self._write(build_command(msg.group, msg.command, msg.speed))

    def _write(self, data):
        try:
            self.serial.write(data)
        except Exception as e:
            self.get_logger().warn(f'串口写入失败：{e}',
                                   throttle_duration_sec=2.0)

    # ---------- 仿真状态机 ----------
    def _sim_command(self, group, command):
        idxs = (0, 1) if group == 0 else (group - 1,)
        if command == CMD_STOP:
            idxs = (0, 1)  # 固件 STOP 是全局的
        for i in idxs:
            if command == CMD_IN and self.pos_mm[i] < self.travel_mm:
                self.state[i] = STATE_MOVING_IN
                self.enabled[i] = True
            elif command == CMD_OUT and self.pos_mm[i] > 0.0:
                self.state[i] = STATE_MOVING_OUT
                self.enabled[i] = True
            elif command == CMD_STOP:
                if self.state[i] in (STATE_MOVING_IN, STATE_MOVING_OUT):
                    self.state[i] = STATE_HOLD_MID
            elif command == CMD_RELAX:
                self.enabled[i] = False
            elif command == CMD_LOCK:
                self.enabled[i] = True

    def _sim_tick(self, dt):
        for i in (0, 1):
            if self.state[i] == STATE_MOVING_IN:
                self.pos_mm[i] = min(self.travel_mm,
                                     self.pos_mm[i] + self.sim_speed * dt)
                if self.pos_mm[i] >= self.travel_mm:
                    self.state[i] = STATE_AT_INNER
                    self.get_logger().info(f'[sim] DONE G{i+1} IN')
            elif self.state[i] == STATE_MOVING_OUT:
                self.pos_mm[i] = max(0.0, self.pos_mm[i] - self.sim_speed * dt)
                if self.pos_mm[i] <= 0.0:
                    self.state[i] = STATE_AT_OUTER
                    self.get_logger().info(f'[sim] DONE G{i+1} OUT')

    # ---------- 上行 ----------
    def _read_serial(self):
        if self.serial is None or not self.serial.is_open:
            return
        try:
            data = self.serial.read(512)
        except Exception as e:
            self.get_logger().warn(f'串口读取异常：{e}',
                                   throttle_duration_sec=2.0)
            return
        if not data:
            return
        self.rx_buf += data.decode('ascii', errors='replace')
        while '\n' in self.rx_buf:
            line, self.rx_buf = self.rx_buf.split('\n', 1)
            self._handle_line(line)

    def _handle_line(self, line):
        ev = parse_line(line)
        if ev is None:
            return
        if ev['type'] == 'status':
            i = ev['group'] - 1
            self.state[i] = ev['state_id']
            self.pos_mm[i] = ev['pos_mm']
            self.enabled[i] = ev['enabled']
        elif ev['type'] == 'done':
            self.pos_mm[ev['group'] - 1] = ev['pos_mm']
            self.get_logger().info(
                f"G{ev['group']} {ev['verb']} 到位 pos={ev['pos_mm']:.2f}mm")
        elif ev['type'] == 'speed':
            self.speed_steps = ev['steps_per_sec']
        elif ev['type'] == 'err':
            self.get_logger().warn(f"下位机拒绝指令：{ev['text']}",
                                   throttle_duration_sec=2.0)

    # ---------- 定时 ----------
    def timer_cb(self):
        now = time.monotonic()
        if self.simulate:
            self._sim_tick(0.1)
            self._publish_sim_joints()
        else:
            self._read_serial()
            if now - self.last_pos_query >= self.status_period:
                self.last_pos_query = now
                self._write(b'POS\n')
        if now - self.last_pub >= 0.5:  # 2Hz 状态发布
            self.last_pub = now
            self._publish_status()

    def _publish_sim_joints(self):
        """仿真位置 → Gazebo 推杆关节指令（q=0 外侧，-travel 内侧）。

        与 model.sdf 棱柱关节限位 [-0.057, 0] 对应；关节控制器跟踪
        连续更新的目标位置，推杆运动速度与 sim_speed_mm_s 一致。
        """
        for grp, pubs in self.sim_joint_pubs.items():
            q = Float64()
            q.data = -self.pos_mm[grp] / 1000.0
            for pub in pubs:
                pub.publish(q)

    def _publish_status(self):
        msg = LeadscrewStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = [int(s) for s in self.state]
        msg.pos_mm = [float(p) for p in self.pos_mm]
        msg.enabled = [bool(e) for e in self.enabled]
        msg.speed = int(self.speed_steps)
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LeadscrewDriverNode()
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
