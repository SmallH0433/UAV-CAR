#!/usr/bin/env python3
"""交互控制 CAR_ws 的 42 步进电机。方向 0/1 是 DIR 电平。"""

import argparse
import fcntl
import math
import os
from pathlib import Path
import signal
import sys
import time

GPIO_ROOT = Path("/sys/class/gpio")
MOTORS = {1: (17, 27, 22), 2: (23, 24, 5)}  # BCM: STEP, DIR, EN
# EN+ connects to GPIO, EN- to GND. Restore the pre-regression run polarity; verify physically.
EN_RUN = 0
EN_RESET = 1
LOCK_FILE = "/tmp/car_leadscrew_manual.lock"
MAX_RPM = 1000.0  # 用户可设的转速上限；高频输出不保证达到设定值


class Output:
    def __init__(self, pin, initial):
        self.path = GPIO_ROOT / f"gpio{pin}"
        if not self.path.exists():
            (GPIO_ROOT / "export").write_text(str(pin))
        deadline = time.monotonic() + 3.0
        while not os.access(self.path / "direction", os.W_OK):
            if time.monotonic() >= deadline:
                raise PermissionError(f"GPIO{pin} 不可写；检查 ubuntu 是否在 gpio 组")
            time.sleep(0.01)
        # direction 写 high/low 可直接设置输出初值，避免使能脚短暂跳变。
        (self.path / "direction").write_text("high" if initial else "low")
        self.fd = os.open(self.path / "value", os.O_WRONLY)

    def write(self, value):
        os.lseek(self.fd, 0, os.SEEK_SET)
        os.write(self.fd, b"1" if value else b"0")

    def close(self):
        os.close(self.fd)


def wait(seconds):
    deadline = time.perf_counter() + seconds
    while True:
        remaining = deadline - time.perf_counter()
        if remaining <= 0:
            return
        if remaining > 0.001:
            time.sleep(remaining - 0.0003)


def pulse_hz(index, count, target, ramp, start):
    if ramp == 0:
        return target
    ratio = min(1.0, (index + 1) / ramp, (count - index) / ramp)
    smooth = ratio * ratio * (3.0 - 2.0 * ratio)
    return start + (target - start) * smooth


def check_other_motor_processes():
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            cmd = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="ignore")
        except (OSError, PermissionError):
            continue
        if any(name in cmd for name in ("leadscrew_driver_node", "cl42_single_motor.py",
                                         "cl42_two_motors.py", "cl42_one_revolution.py")):
            raise RuntimeError(f"发现另一电机控制进程 PID {entry.name}；先停止它再运行本脚本")


def move(motor, direction, rpm, distance_mm, pulses_per_rev, pitch_mm,
         ramp_seconds=0.1, dry_run=False):
    if direction not in (0, 1):
        raise ValueError("方向只能是 0（DIR 低）或 1（DIR 高）")
    if not math.isfinite(rpm) or not 0 < rpm <= MAX_RPM:
        raise ValueError(f"转速必须大于 0 且不超过 {MAX_RPM:g} rpm")
    if not math.isfinite(distance_mm) or not 0 < distance_mm <= 62:
        raise ValueError("本次行程必须在 0～62 mm 之间")
    if pulses_per_rev <= 0 or not math.isfinite(pitch_mm) or pitch_mm <= 0:
        raise ValueError("每圈脉冲数和丝杆导程必须大于 0")
    if not math.isfinite(ramp_seconds) or not 0.05 <= ramp_seconds <= 5:
        raise ValueError("加速时间须在 0.05～5 秒之间")
    hz = rpm * pulses_per_rev / 60.0
    if hz < 5:
        raise ValueError(f"当前设置产生 {hz:.1f} Hz，至少需要 5 Hz；请提高转速")
    pulses = round(distance_mm * pulses_per_rev / pitch_mm)
    if pulses < 1:
        raise ValueError("行程太小，换算后不到 1 个脉冲")
    start_hz = min(hz, max(5.0, hz * 0.10))
    ramp_pulses = min(pulses // 2, max(1, round(ramp_seconds * (start_hz + hz) / 2)))
    step_pin, dir_pin, en_pin = MOTORS[motor]
    print(f"电机{motor}: STEP=GPIO{step_pin}, DIR=GPIO{dir_pin}({direction}), EN=GPIO{en_pin}")
    print(f"转速={rpm:g} rpm, 目标频率={hz:.1f} Hz, 行程={distance_mm:g} mm, "
          f"脉冲={pulses}, 理论行程={pulses * pitch_mm / pulses_per_rev:.3f} mm, "
          f"加速时间设置={ramp_seconds:g} s, 加速段={ramp_pulses} 脉冲", flush=True)
    if dry_run:
        print("参数检查完成；未操作 GPIO")
        return

    check_other_motor_processes()
    step = direction_gpio = enable = None
    sent = 0
    started = None
    with open(LOCK_FILE, "a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            step = Output(step_pin, 0)
            enable = Output(en_pin, EN_RUN)  # Keep run level; reset is an explicit separate operation.
            direction_gpio = Output(dir_pin, direction)
            time.sleep(0.2)
            enable.write(EN_RUN)
            time.sleep(0.2)
            started = time.perf_counter()
            for index in range(pulses):
                half_period = 0.5 / pulse_hz(index, pulses, hz, ramp_pulses, start_hz)
                step.write(1)
                sent += 1
                wait(half_period)
                step.write(0)
                wait(half_period)
        finally:
            elapsed = time.perf_counter() - started if started is not None else None
            if step is not None:
                step.write(0)
            if enable is not None:
                enable.write(EN_RUN)  # 停止后保持使能/自锁
            for gpio in (step, direction_gpio, enable):
                if gpio is not None:
                    gpio.close()
            print(f"已发送 {sent}/{pulses} 个脉冲；STEP=低，EN=低（运行电平）", flush=True)
            if elapsed and sent:
                print(f"实际用时={elapsed:.2f} s，整段平均脉冲频率="
                      f"{sent / elapsed:.1f} Hz（含加减速）", flush=True)


def ask(text, convert):
    while True:
        answer = input(text).strip()
        if answer.lower() in ("q", "quit", "exit"):
            return None
        try:
            return convert(answer)
        except ValueError:
            print("输入无效，请重试；输入 q 退出。")


def main():
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    for name in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(name, interrupted)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor", type=int, choices=(1, 2), default=1)
    parser.add_argument("--direction", type=int, choices=(0, 1),
                        help="DIR 电平：1号电机 0=向内/低、1=向外/高；2号镜像反向")
    parser.add_argument("--rpm", type=float, help="电机转速，单位 rpm")
    parser.add_argument("--distance-mm", type=float, help="本次相对行程，单位 mm，最大 62")
    parser.add_argument("--pulses-per-rev", type=int, default=1600)
    parser.add_argument("--pitch-mm", type=float, default=2.0)
    parser.add_argument("--ramp-seconds", type=float, default=0.1,
                        help="加速段目标时长，默认 0.1 秒，范围 0.05～5 秒；实际时长可能不同")
    parser.add_argument("--dry-run", action="store_true", help="仅检查参数，不操作 GPIO")
    args = parser.parse_args()

    try:
        if args.direction is not None and args.rpm is not None and args.distance_mm is not None:
            move(args.motor, args.direction, args.rpm, args.distance_mm,
                 args.pulses_per_rev, args.pitch_mm, args.ramp_seconds, args.dry_run)
            return 0
        print("交互模式：1号电机向内=0/低、向外=1/高；2号电机镜像反向。输入 q 退出。")
        while True:
            motor = ask("电机 [1/2，默认 1]：", lambda s: int(s or "1"))
            if motor is None:
                return 0
            if motor not in MOTORS:
                print("电机只能选 1 或 2")
                continue
            direction = ask("方向 [0=DIR 低/1=DIR 高]：", int)
            if direction is None:
                return 0
            rpm = ask("转速 [rpm，建议先输入 6]：", float)
            if rpm is None:
                return 0
            distance = ask("本次行程 [mm，建议先输入 1]：", float)
            if distance is None:
                return 0
            try:
                move(motor, direction, rpm, distance, args.pulses_per_rev,
                     args.pitch_mm, args.ramp_seconds, args.dry_run)
            except (ValueError, RuntimeError, OSError, BlockingIOError) as exc:
                print(f"本次操作失败：{exc}", file=sys.stderr)
    except KeyboardInterrupt:
        print("\n已中断并停止步进脉冲。", file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, OSError, BlockingIOError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
