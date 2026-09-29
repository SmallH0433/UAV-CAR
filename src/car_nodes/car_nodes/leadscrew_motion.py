"""Single-owner CL42 pulse engine shared by all ROS commands. No alarm/limit feedback."""
import fcntl
import json
import math
import os
from pathlib import Path
import threading
import time

LOCK_FILE = '/tmp/car_leadscrew_manual.lock'
DEFAULT_PINS = ((17, 27, 22), (23, 24, 5))
PULSES_PER_REV = 1600
PITCH_MM = 2.0
SIM_STATE_FILE = Path.home() / '.ros' / 'leadscrew_simulation_state.json'


def centered_geometry(open_gap_mm, target_gap_mm):
    """Return exact symmetric travel for a bidirectional leadscrew."""
    travel_mm = (open_gap_mm - target_gap_mm) / 2.0
    turns = travel_mm / PITCH_MM
    return dict(open_gap_mm=open_gap_mm, target_gap_mm=target_gap_mm,
                travel_mm=travel_mm, turns=turns,
                pulses=round(turns * PULSES_PER_REV))


SIM_GEOMETRY = {
    1: centered_geometry(262.0, 160.0),
    2: centered_geometry(297.0, 170.0),
}


class Output:
    def __init__(self, pin, value):
        root = Path('/sys/class/gpio')
        path = root / f'gpio{pin}'
        if not path.exists():
            (root / 'export').write_text(str(pin))
        deadline = time.monotonic() + 3
        while not os.access(path / 'direction', os.W_OK):
            if time.monotonic() > deadline:
                raise RuntimeError(f'GPIO{pin} 权限不可用，请检查 gpio 用户组及 udev')
            time.sleep(.01)
        (path / 'direction').write_text('high' if value else 'low')
        self.fd = os.open(path / 'value', os.O_WRONLY)

    def write(self, value):
        os.lseek(self.fd, 0, os.SEEK_SET)
        os.write(self.fd, b'1' if value else b'0')

    def close(self):
        os.close(self.fd)  # Preserve final levels; never unexport EN on shutdown.


class Motion:
    def __init__(self, simulate=False, pins=DEFAULT_PINS, inversions=(False, False),
                 enable_inversions=(False, False), pulses_per_rev=1600, pitch=2., travel=63.5,
                 output_factory=Output):
        self.simulate = simulate
        self.pins = pins
        self.inversions = inversions
        self.enable_inversions = enable_inversions
        if pulses_per_rev <= 0 or not math.isfinite(pitch) or pitch <= 0 or not 0 < travel <= 63.5:
            raise ValueError('丝杆机械参数无效')
        flat = [p for row in pins for p in row]
        if len(set(flat)) != 6 or any(type(p) is not int or not 0 <= p <= 27 for p in flat):
            raise ValueError('GPIO 必须为 6 个互不重复的 BCM 引脚')
        self.ppr, self.pitch, self.travel = pulses_per_rev, pitch, travel
        self.steps_mm = pulses_per_rev / pitch
        self.gate, self.lock = threading.RLock(), threading.RLock()
        self.cancel = threading.Event()
        self.worker = None
        self.busy = False
        self.resetting = False
        self.closed = False
        self.pos = [0, 0]  # Relative to node start, not a measured physical position.
        self.states = [4, 4]
        self.enabled = [True, True]
        self.speed = 0
        self.message = '已就绪；位置为启动后的相对脉冲估算'
        self.output = ''
        self.simulation_clamped = self._load_simulation_state()
        self.gpios = []
        self.ownership = None
        try:
            if not simulate:
                self.ownership = open(LOCK_FILE, 'a')
                fcntl.flock(self.ownership, fcntl.LOCK_EX | fcntl.LOCK_NB)
                for i, (step, direction, enable) in enumerate(pins):
                    row = []
                    self.gpios.append(row)
                    for pin, value in ((step, 0), (direction, 0), (enable, self.en_level(i, True))):
                        row.append(output_factory(pin, value))
        except Exception:
            self.close()
            raise

    def en_level(self, i, enabled):
        return (0 if enabled else 1) ^ int(self.enable_inversions[i])

    @staticmethod
    def _load_simulation_state():
        try:
            data = json.loads(SIM_STATE_FILE.read_text(encoding='utf-8'))
            return bool(data['clamped'])
        except (OSError, ValueError, KeyError, TypeError):
            return False

    def _save_simulation_state(self):
        SIM_STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
        temporary = SIM_STATE_FILE.with_suffix('.tmp')
        temporary.write_text(json.dumps({'clamped': self.simulation_clamped}),
                             encoding='utf-8')
        temporary.replace(SIM_STATE_FILE)

    def write(self, i, column, value):
        if not self.simulate:
            self.gpios[i][column].write(value)

    @staticmethod
    def indices(group):
        if type(group) is not int or group not in (0, 1, 2):
            raise ValueError('电机选择必须为 0（两台）、1 或 2')
        return (0, 1) if group == 0 else (group - 1,)

    def snapshot(self):
        with self.lock:
            return dict(available=True, simulated=self.simulate, moving=self.busy and not self.resetting,
                        busy=self.busy, resetting=self.resetting, message=self.message, output=self.output,
                        pos_mm=[round(p / self.steps_mm, 3) for p in self.pos],
                        state=list(self.states), enabled=list(self.enabled), speed=self.speed,
                        position_reference='relative_to_node_start', alarm_feedback=False,
                        simulation_clamped=self.simulation_clamped,
                        simulation_next_action=('release' if self.simulation_clamped else 'clamp'),
                        simulation_geometry=SIM_GEOMETRY,
                        pins=[dict(step=p[0], direction=p[1], enable=p[2]) for p in self.pins])

    def launch(self, task, resetting, message):
        with self.gate, self.lock:
            if self.closed:
                raise RuntimeError('丝杆驱动已关闭')
            if self.busy:
                raise RuntimeError('丝杆正在运动或复位，请先停止')
            self.cancel.clear()
            self.busy, self.resetting = True, resetting
            self.message, self.output = message, ''
            self.worker = threading.Thread(target=self.run, args=(task,), daemon=True)
            self.worker.start()

    def run(self, task):
        try:
            task()
            with self.lock:
                self.message = ('已停止' if self.cancel.is_set() else
                                'EN 复位序列已完成，请观察报警灯' if self.resetting else
                                '脉冲输出完成；实际位置需观察确认')
        except Exception as exc:
            with self.lock:
                self.message = '执行失败：' + str(exc)
        finally:
            errors = []
            for i in (0, 1):
                try:
                    self.write(i, 0, 0)
                except Exception as exc:
                    errors.append(str(exc))
            with self.lock:
                self.states = [4, 4]
                self.busy = self.resetting = False
                if errors:
                    self.message = '停止脉冲输出失败：' + '; '.join(errors)

    def move(self, group, direction, rpm, distance, ramp):
        idxs = self.indices(group)
        if type(direction) is not int or direction not in (0, 1):
            raise ValueError('方向必须为 0（向内）或 1（向外）')
        if not math.isfinite(rpm) or not 0 < rpm <= 1000 or rpm * self.ppr / 60 < 5:
            raise ValueError('转速必须对应至少 5 Hz 且不超过 1000 rpm')
        if not math.isfinite(distance) or not 0 < distance <= self.travel:
            raise ValueError(f'单次行程须大于 0 且不超过 {self.travel:g} mm')
        if not math.isfinite(ramp) or not .05 <= ramp <= 5:
            raise ValueError('加速时间须在 0.05–5 秒内')
        count = round(distance * self.steps_mm)
        if count < 1:
            raise ValueError('行程不足一个脉冲')
        hz = rpm * self.ppr / 60
        self.launch(lambda: self.pulses(idxs, direction, count, hz, ramp), False,
                    f'电机{group or "1+2"}：{rpm:g} rpm，{distance:g} mm')

    def delay(self, seconds):
        deadline = time.perf_counter() + seconds
        while True:
            left = deadline - time.perf_counter()
            if left <= 0:
                return
            if left > .001:
                time.sleep(left - .0003)

    def pulses(self, idxs, direction, count, hz, ramp):
        sent = 0
        started = time.monotonic()
        for i in idxs:
            self.write(i, 0, 0)
            self.write(i, 1, direction ^ int(self.inversions[i]))
            self.write(i, 2, self.en_level(i, True))
        with self.lock:
            self.speed = round(hz)
            for i in idxs:
                self.enabled[i] = True
                self.states[i] = 1 if direction == 0 else 3
        if self.cancel.wait(.2):
            return
        start = min(hz, max(5., hz * .1))
        ramp_count = min(count // 2, max(1, round(ramp * (start + hz) / 2)))
        try:
            for n in range(count):
                if self.cancel.is_set():
                    break
                ratio = min(1., (n + 1) / ramp_count, (count - n) / ramp_count) if ramp_count else 1.
                frequency = start + (hz - start) * ratio * ratio * (3 - 2 * ratio)
                half = .5 / frequency
                for i in idxs:
                    self.write(i, 0, 1)
                self.delay(half)
                for i in idxs:
                    self.write(i, 0, 0)
                sent += 1
                with self.lock:
                    for i in idxs:
                        self.pos[i] += 1 if direction == 0 else -1
                self.delay(half)  # Relative deadlines guarantee no catch-up pulse bursts.
        finally:
            with self.lock:
                self.output = f'已输出 {sent}/{count} 个脉冲，用时 {time.monotonic()-started:.2f} 秒；无编码器/限位反馈'

    def landing_sequence(self, direction=0):
        if direction not in (0,1): raise ValueError("Invalid sequence direction")
        def task():
            reports=[]
            for i in (0,1):
                if self.cancel.is_set(): break
                with self.lock:
                    self.states=[4,4]
                    self.message=f'Sequence: motor {i+1}, direction={direction}, 50mm, 100rpm, ramp 0.1s'
                self.pulses((i,),direction,round(50*self.steps_mm),100*self.ppr/60,.1)
                reports.append(f'Motor {i+1}: '+self.output)
            with self.lock: self.output='\n'.join(reports)
        self.launch(task,False,'Landing sequence: motor 1 then motor 2')

    def simulation_toggle(self, rpm, ramp):
        """Toggle the fixed UAV centering sequence; stages never overlap."""
        if not math.isfinite(rpm) or not 0 < rpm <= 1000 or rpm * self.ppr / 60 < 5:
            raise ValueError('转速必须对应至少 5 Hz 且不超过 1000 rpm')
        if not math.isfinite(ramp) or not .05 <= ramp <= 5:
            raise ValueError('加速时间必须在 0.05–5 秒之间')
        with self.lock:
            clamp = not self.simulation_clamped
        if clamp:
            stages = ((1, 0, SIM_GEOMETRY[2]['travel_mm'], '2号电机先夹紧'),
                      (0, 0, SIM_GEOMETRY[1]['travel_mm'], '1号电机随后夹紧'))
        else:
            stages = ((0, 1, SIM_GEOMETRY[1]['travel_mm'], '1号电机先松开'),
                      (1, 1, SIM_GEOMETRY[2]['travel_mm'], '2号电机随后松开'))

        def task():
            reports = []
            for index, (motor, direction, distance, label) in enumerate(stages, 1):
                if self.cancel.is_set():
                    break
                with self.lock:
                    self.message = f'{label}（第 {index}/2 步）'
                self.pulses((motor,), direction, round(distance * self.steps_mm),
                            rpm * self.ppr / 60, ramp)
                with self.lock:
                    reports.append(f'【{label}】\n{self.output}')
                if self.cancel.is_set():
                    break
            with self.lock:
                self.output = '\n\n'.join(reports)
                if not self.cancel.is_set() and len(reports) == len(stages):
                    self.simulation_clamped = clamp
                    self._save_simulation_state()

        action = '归中' if clamp else '释放'
        self.launch(task, False, f'仿真模式正在{action}：准备执行顺序动作')

    def reset(self, group):
        idxs = self.indices(group)
        def task():
            try:
                for i in idxs:
                    self.write(i, 0, 0)
                    self.write(i, 2, self.en_level(i, False))
                with self.lock:
                    for i in idxs:
                        self.enabled[i] = False
                self.cancel.wait(1.)
            finally:
                for i in idxs:
                    self.write(i, 2, self.en_level(i, True))
                with self.lock:
                    for i in idxs:
                        self.enabled[i] = True
                    self.output = 'EN 禁用后已恢复运行电平；未发运动脉冲，未确认报警消除'
        self.launch(task, True, '正在执行 EN 复位…')

    def stop(self):
        with self.gate:
            self.cancel.set()
            if self.worker is not None:
                self.worker.join(timeout=3)
                if self.worker.is_alive():
                    raise RuntimeError('停止未完成，禁止继续启动')

    def enable(self, group, value):
        with self.gate:
            self.stop()
            for i in self.indices(group):
                self.write(i, 2, self.en_level(i, value))
                with self.lock:
                    self.enabled[i] = value

    def close(self):
        with self.gate:
            self.stop()
            self.closed = True
            for row in self.gpios:
                if row:
                    row[0].write(0)
                for output in row:
                    output.close()
            self.gpios = []
            if self.ownership is not None:
                self.ownership.close()
                self.ownership = None
