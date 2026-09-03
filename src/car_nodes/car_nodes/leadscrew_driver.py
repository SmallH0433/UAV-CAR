"""双向丝杆驱动节点（树莓派 4B GPIO 直连双 42 步进 + T8 双向丝杆）

订阅：
  /leadscrew/cmd    (car_interfaces/LeadscrewCommand)  控制组指令
发布：
  /leadscrew/status (car_interfaces/LeadscrewStatus, 2Hz + 状态变化时)
  /leadscrew/sim/pusher_{a,b,c,d}/cmd_pos (std_msgs/Float64, 10Hz,
    仅 simulate=true 且 publish_sim_joints=true：驱动 Gazebo 模型
    r680_4wd 的 4 个推杆棱柱关节；组1→pusher_a/pusher_c，
    组2→pusher_b/pusher_d。q=0 螺母在外侧(pos=0)，q=-0.067 到内侧
    (pos=travel_mm)，与 model.sdf 关节限位一致)
参数：
  simulate          (bool,  True)           True=本地模拟状态机（不操作 GPIO）
  publish_sim_joints (bool, True)           仿真时同步发布 Gazebo 推杆关节指令
  status_period     (float, 2.0)            状态发布周期 s（保留参数，当前固定 0.5s）
  sim_speed_mm_s    (float, 2.0)            仿真模式下的螺母移动速度 mm/s
  travel_mm         (float, 67.0)           单边行程 mm
  leadscrew_pitch_mm (float, 2.0)           丝杆导程 mm/圈
  pulses_per_rev    (int,   1600)           电机转一圈所需脉冲（CL42 微步设置）
  ramp_seconds      (float, 1.0)            加减速斜坡时间 s
  min_start_hz      (float, 5.0)            起步频率 Hz
  default_speed     (int,   1600)           LeadscrewCommand.speed=0 时的默认脉冲频率
  step_pin_1        (int,   17)             电机 1 STEP (PUL+)
  dir_pin_1         (int,   27)             电机 1 DIR  (DIR+)
  enable_pin_1      (int,   13)             电机 1 EN+，-1=未接线
  dir_invert_1      (bool,  False)           电机 1 方向取反
  enable_invert_1   (bool,  False)           电机 1 EN 电平取反（默认低=使能）
  step_pin_2        (int,   23)             电机 2 STEP (PUL+)
  dir_pin_2         (int,   24)             电机 2 DIR  (DIR+)
  enable_pin_2      (int,   5)              电机 2 EN+，-1=未接线
  dir_invert_2      (bool,  True)            电机 2 方向取反（实机安装方向与电机 1 镜像）
  enable_invert_2   (bool,  False)           电机 2 EN 电平取反（默认低=使能）

硬件接线（共阴极，PUL-/DIR-/EN- 接 GND）：
  电机 1：GPIO17(pin11) → PUL+，GPIO27(pin13) → DIR+，GPIO13(pin33) → EN+
  电机 2：GPIO23(pin16) → PUL+，GPIO24(pin18) → DIR+，GPIO5(pin29)  → EN+
  CL42 常见逻辑：EN 悬空或低电平=使能，高电平=释放（RELAX）。
  若实际相反，把对应 enable_invert_* 设为 True。

位置为开环计数，节点启动时默认螺母在外侧（pos=0）。若实际不在外侧，请先手动归位。
"""

import os
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64

from car_interfaces.msg import LeadscrewCommand, LeadscrewStatus
from car_nodes.esp_leadscrew_protocol import (
    CMD_STOP, CMD_IN, CMD_OUT, CMD_RELAX, CMD_LOCK)

STATE_AT_OUTER, STATE_MOVING_IN, STATE_AT_INNER, STATE_MOVING_OUT, \
    STATE_HOLD_MID = range(5)

# 控制组 → Gazebo 推杆关节映射（r680_4wd/model.sdf）：
# 组1 = 电机A（A 边中点下方）→ pusher_a/pusher_c；组2 = 电机B → pusher_b/pusher_d
SIM_JOINT_TOPICS = {0: ('/leadscrew/sim/pusher_a/cmd_pos',
                        '/leadscrew/sim/pusher_c/cmd_pos'),
                    1: ('/leadscrew/sim/pusher_b/cmd_pos',
                        '/leadscrew/sim/pusher_d/cmd_pos')}

GPIO_ROOT = Path('/sys/class/gpio')


class SysfsGpio:
    """通过 Linux sysfs 操作 Raspberry Pi GPIO 输出（零依赖）。"""

    def __init__(self, pin: int, initial: int = 0):
        self.pin = pin
        self.path = GPIO_ROOT / f'gpio{pin}'
        self.exported_here = False
        self.fd = None

        if not GPIO_ROOT.exists():
            raise RuntimeError('Linux GPIO sysfs 接口不可用')

        if not self.path.exists():
            (GPIO_ROOT / 'export').write_text(str(pin))
            self.exported_here = True
            for _ in range(100):
                if self.path.exists():
                    break
                time.sleep(0.001)
            else:
                raise RuntimeError(f'GPIO{pin} export 超时')

        # export 后 udev 规则异步 chgrp 到 gpio 组，需等权限就绪再写，
        # 否则会报 Permission denied（udev 处理有几~几十毫秒延迟）
        for _ in range(300):
            if os.access(self.path / 'direction', os.W_OK):
                break
            time.sleep(0.01)
        else:
            raise RuntimeError(
                f'GPIO{pin} 等待 udev 权限就绪超时（请检查 99-car-devices.rules '
                f'是否已安装到 /etc/udev/rules.d/ 并 udevadm trigger）')

        (self.path / 'direction').write_text('out')
        self.fd = os.open(self.path / 'value', os.O_WRONLY)
        self.write(initial)

    def write(self, value: int) -> None:
        if self.fd is None:
            return
        os.lseek(self.fd, 0, os.SEEK_SET)
        os.write(self.fd, b'1' if value else b'0')

    def close(self) -> None:
        if self.fd is not None:
            try:
                self.write(0)
            finally:
                os.close(self.fd)
                self.fd = None
        if self.exported_here:
            try:
                (GPIO_ROOT / 'unexport').write_text(str(self.pin))
            except OSError:
                pass


def wait_until(deadline: float) -> None:
    """以绝对单调时钟等待，降低脉冲抖动；接近截止线前让出 GIL，
    避免步进线程饿死 ROS 执行器。"""
    while True:
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            return
        if remaining > 0.001:
            time.sleep(remaining - 0.0005)
        elif remaining > 0.0002:
            # 剩余 0.2~1ms 时先睡掉大部分，最后 0.2ms 忙等精确定位
            time.sleep(remaining - 0.00015)


def smoothstep(value: float) -> float:
    return value * value * (3.0 - 2.0 * value)


def pulse_frequency(
    pulse_index: int,
    total_pulses: int,
    ramp_pulses: int,
    start_hz: float,
    target_hz: float,
) -> float:
    """根据当前脉冲索引计算瞬时频率（S 形加减速）。"""
    if ramp_pulses <= 0:
        return target_hz
    if pulse_index < ramp_pulses:
        ratio = smoothstep((pulse_index + 1) / ramp_pulses)
        return start_hz + (target_hz - start_hz) * ratio
    remaining = total_pulses - pulse_index
    if remaining <= ramp_pulses:
        ratio = smoothstep(max(0.0, (remaining - 1) / ramp_pulses))
        return start_hz + (target_hz - start_hz) * ratio
    return target_hz


class LeadscrewDriverNode(Node):
    def __init__(self):
        super().__init__('leadscrew_driver_node')
        self.declare_parameter('port', '')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('simulate', True)
        self.declare_parameter('publish_sim_joints', True)
        self.declare_parameter('status_period', 2.0)
        self.declare_parameter('sim_speed_mm_s', 2.0)
        self.declare_parameter('travel_mm', 67.0)
        self.declare_parameter('leadscrew_pitch_mm', 2.0)
        self.declare_parameter('pulses_per_rev', 1600)
        self.declare_parameter('ramp_seconds', 1.0)
        self.declare_parameter('min_start_hz', 5.0)
        self.declare_parameter('default_speed', 1600)
        self.declare_parameter('step_pin_1', 17)
        self.declare_parameter('dir_pin_1', 27)
        self.declare_parameter('enable_pin_1', 13)
        self.declare_parameter('dir_invert_1', False)
        self.declare_parameter('enable_invert_1', False)
        self.declare_parameter('step_pin_2', 23)
        self.declare_parameter('dir_pin_2', 24)
        self.declare_parameter('enable_pin_2', 5)
        self.declare_parameter('dir_invert_2', True)
        self.declare_parameter('enable_invert_2', False)

        self.simulate = self.get_parameter('simulate').value
        self.travel_mm = self.get_parameter('travel_mm').value
        self.leadscrew_pitch_mm = self.get_parameter('leadscrew_pitch_mm').value
        self.pulses_per_rev = self.get_parameter('pulses_per_rev').value
        self.ramp_seconds = self.get_parameter('ramp_seconds').value
        self.min_start_hz = self.get_parameter('min_start_hz').value
        self.default_speed = int(self.get_parameter('default_speed').value)
        self.status_period = self.get_parameter('status_period').value
        self.sim_speed = self.get_parameter('sim_speed_mm_s').value

        self.steps_per_mm = self.pulses_per_rev / self.leadscrew_pitch_mm
        self.travel_steps = int(round(self.travel_mm * self.steps_per_mm))

        # 两组状态缓存（下标 0=组1 1=组2）
        self.state = [STATE_AT_OUTER, STATE_AT_OUTER]
        self.pos_mm = [0.0, 0.0]
        self.enabled = [True, True]
        self.speed_steps = self.default_speed

        # 运动控制共享状态
        self.lock = threading.Lock()
        self._wake = threading.Event()
        self.current_steps = [0, 0]
        self.target_steps = [0, 0]
        self.motion_dir = [0, 0]
        self.motion_total = [0, 0]
        self.motion_stepped = [0, 0]
        self.motion_active = [False, False]
        self.last_dir = [None, None]

        # GPIO 资源
        self.gpio_ok = False
        self.step_gpios = [None, None]
        self.dir_gpios = [None, None]
        self.enable_gpios = [None, None]
        self.dir_invert = [
            self.get_parameter('dir_invert_1').value,
            self.get_parameter('dir_invert_2').value,
        ]
        self.enable_invert = [
            self.get_parameter('enable_invert_1').value,
            self.get_parameter('enable_invert_2').value,
        ]

        self._worker_running = True
        self.worker = None

        if not self.simulate:
            try:
                self._init_gpio()
                self.gpio_ok = True
                pin1 = (self.get_parameter('step_pin_1').value,
                        self.get_parameter('dir_pin_1').value)
                pin2 = (self.get_parameter('step_pin_2').value,
                        self.get_parameter('dir_pin_2').value)
                en1 = self.get_parameter('enable_pin_1').value
                en2 = self.get_parameter('enable_pin_2').value
                self.get_logger().info(
                    f'GPIO 直连双电机已就绪：M1=PUL{pin1[0]}/DIR{pin1[1]}/EN{en1} '
                    f'M2=PUL{pin2[0]}/DIR{pin2[1]}/EN{en2}，行程 {self.travel_mm:g}mm '
                    f'({self.travel_steps} 步)')
                self.worker = threading.Thread(target=self._step_worker, daemon=True)
                self.worker.start()
            except Exception as e:
                self.get_logger().error(f'GPIO 初始化失败：{e}')
        else:
            self.get_logger().info('仿真模式：本地模拟状态机，不操作 GPIO')

        # 仿真模式同步驱动 Gazebo 推杆关节
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

    # ---------- GPIO 初始化与清理 ----------
    def _init_gpio(self):
        pins = [
            (self.get_parameter('step_pin_1').value, self.get_parameter('dir_pin_1').value,
             self.get_parameter('enable_pin_1').value),
            (self.get_parameter('step_pin_2').value, self.get_parameter('dir_pin_2').value,
             self.get_parameter('enable_pin_2').value),
        ]
        for i, (step_pin, dir_pin, en_pin) in enumerate(pins):
            self.step_gpios[i] = SysfsGpio(step_pin, 0)
            self.dir_gpios[i] = SysfsGpio(dir_pin, 0)
            if en_pin >= 0:
                self.enable_gpios[i] = SysfsGpio(en_pin, self._en_level(True, i))

    def _en_level(self, enabled: bool, idx: int = 0) -> int:
        # 默认 EN 低电平=使能（等效 EN 悬空），高电平=释放
        level = 0 if enabled else 1
        return level ^ 1 if self.enable_invert[idx] else level

    def _apply_enable(self, idx: int) -> None:
        g = self.enable_gpios[idx]
        if g is not None:
            g.write(self._en_level(self.enabled[idx], idx))

    def _close_gpio(self):
        for g in self.step_gpios + self.dir_gpios + self.enable_gpios:
            if g is not None:
                try:
                    g.close()
                except Exception:
                    pass
        self.step_gpios = [None, None]
        self.dir_gpios = [None, None]
        self.enable_gpios = [None, None]

    # ---------- 下行 ----------
    def cmd_cb(self, msg):
        self.get_logger().info(f'收到指令 group={msg.group} command={msg.command} speed={msg.speed}')
        if msg.command not in (CMD_STOP, CMD_IN, CMD_OUT, CMD_RELAX, CMD_LOCK):
            self.get_logger().warn(f'未知 command={msg.command}，忽略')
            return
        if msg.speed:
            self.speed_steps = int(msg.speed)
        if self.simulate:
            self._sim_command(msg.group, msg.command)
            return
        if not self.gpio_ok:
            self.get_logger().warn('GPIO 未就绪，指令被丢弃',
                                   throttle_duration_sec=2.0)
            return
        self._gpio_command(msg.group, msg.command)

    def _gpio_command(self, group, command):
        idxs = (0, 1) if group == 0 else (group - 1,)
        with self.lock:
            for i in idxs:
                if command == CMD_IN:
                    self.target_steps[i] = self.travel_steps
                    if self.current_steps[i] < self.target_steps[i]:
                        self._start_motion(i, 1)
                        self.state[i] = STATE_MOVING_IN
                    else:
                        self.state[i] = STATE_AT_INNER
                elif command == CMD_OUT:
                    self.target_steps[i] = 0
                    if self.current_steps[i] > 0:
                        self._start_motion(i, -1)
                        self.state[i] = STATE_MOVING_OUT
                    else:
                        self.state[i] = STATE_AT_OUTER
                elif command == CMD_STOP:
                    self.motion_active[i] = False
                    self.target_steps[i] = self.current_steps[i]
                    self.state[i] = STATE_HOLD_MID
                elif command == CMD_RELAX:
                    self.enabled[i] = False
                    self._apply_enable(i)
                    if self.enable_gpios[i] is None:
                        self.get_logger().warn(
                            f'G{i+1} RELAX：EN 未接线，驱动器仍保持使能',
                            throttle_duration_sec=5.0)
                elif command == CMD_LOCK:
                    self.enabled[i] = True
                    self._apply_enable(i)

    def _start_motion(self, idx: int, direction: int):
        self.motion_dir[idx] = direction
        self.motion_total[idx] = abs(self.target_steps[idx] - self.current_steps[idx])
        self.motion_stepped[idx] = 0
        self.motion_active[idx] = True
        self.enabled[idx] = True
        self._apply_enable(idx)
        self._wake.set()

    # ---------- 步进后台线程 ----------
    def _step_worker(self):
        # 单独线程只检查自己的停止标志；无运动时阻塞等待事件，
        # 不占用 CPU，避免饿死 ROS 订阅/定时器回调。
        while self._worker_running:
            with self.lock:
                active = [i for i in (0, 1) if self.motion_active[i]]

            if not active:
                self._wake.clear()
                self._wake.wait(timeout=0.1)
                continue

            # 设置方向（仅在变化时插入 setup 时间）
            dir_changed = False
            desired_dirs = {}
            with self.lock:
                for i in active:
                    d = 1 if self.motion_dir[i] > 0 else 0
                    if self.dir_invert[i]:
                        d ^= 1
                    desired_dirs[i] = d
                    if self.last_dir[i] != d:
                        self.last_dir[i] = d
                        dir_changed = True

            for i, d in desired_dirs.items():
                self.dir_gpios[i].write(d)
            if dir_changed:
                time.sleep(0.002)  # > CL42 DIR setup/hold 要求

            # 按行程最长的电机统一当前节拍频率
            with self.lock:
                max_total = 0
                max_stepped = 0
                for i in active:
                    if self.motion_total[i] > max_total:
                        max_total = self.motion_total[i]
                        max_stepped = self.motion_stepped[i]

            if max_total == 0:
                with self.lock:
                    for i in active:
                        self.motion_active[i] = False
                        if self.motion_dir[i] > 0:
                            self.state[i] = STATE_AT_INNER
                        elif self.motion_dir[i] < 0:
                            self.state[i] = STATE_AT_OUTER
                continue

            ramp_pulses = self._ramp_pulses(max_total)
            target_hz = max(1.0, float(self.speed_steps))
            start_hz = max(self.min_start_hz, target_hz * 0.10)
            freq = pulse_frequency(max_stepped, max_total, ramp_pulses, start_hz, target_hz)
            half_period = 0.5 / freq

            deadline = time.perf_counter()
            for i in active:
                self.step_gpios[i].write(1)
            deadline += half_period
            wait_until(deadline)
            for i in active:
                self.step_gpios[i].write(0)
            deadline += half_period
            wait_until(deadline)

            # 更新位置并检查到位
            done = []
            with self.lock:
                for i in active:
                    self.motion_stepped[i] += 1
                    self.current_steps[i] += self.motion_dir[i]
                    self.pos_mm[i] = self.current_steps[i] / self.steps_per_mm
                    if self.current_steps[i] == self.target_steps[i]:
                        self.motion_active[i] = False
                        done.append(i)

            for i in done:
                if self.motion_dir[i] > 0:
                    self.state[i] = STATE_AT_INNER
                elif self.motion_dir[i] < 0:
                    self.state[i] = STATE_AT_OUTER
                self.get_logger().info(
                    f'G{i+1} 到位 pos={self.pos_mm[i]:.2f}mm')

    def _ramp_pulses(self, total: int) -> int:
        target_hz = max(1.0, float(self.speed_steps))
        start_hz = max(self.min_start_hz, target_hz * 0.10)
        estimated = int(self.ramp_seconds * (start_hz + target_hz) / 2.0)
        return min(estimated, total // 2)

    # ---------- 仿真状态机 ----------
    def _sim_command(self, group, command):
        idxs = (0, 1) if group == 0 else (group - 1,)
        if command == CMD_STOP:
            idxs = (0, 1)
        with self.lock:
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
        with self.lock:
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
    def timer_cb(self):
        now = time.monotonic()
        if self.simulate:
            self._sim_tick(0.1)
            self._publish_sim_joints()

        if now - self.last_pub >= 0.5:  # 2Hz 状态发布
            self.last_pub = now
            self._publish_status()

    def _publish_sim_joints(self):
        with self.lock:
            pos = list(self.pos_mm)
        for grp, pubs in self.sim_joint_pubs.items():
            q = Float64()
            q.data = -pos[grp] / 1000.0
            for pub in pubs:
                pub.publish(q)

    def _publish_status(self):
        msg = LeadscrewStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.state = [int(s) for s in self.state]
            msg.pos_mm = [float(p) for p in self.pos_mm]
            msg.enabled = [bool(e) for e in self.enabled]
            msg.speed = int(self.speed_steps)
        self.pub_status.publish(msg)

    def destroy_node(self):
        self._worker_running = False
        if self.worker is not None and self.worker.is_alive():
            self.worker.join(timeout=2.0)
        self._close_gpio()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeadscrewDriverNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
