#!/usr/bin/env python3
"""Cycle the selected CL42 EN input without issuing motion pulses."""
import argparse
import fcntl
import signal
import time
from leadscrew_manual import Output, MOTORS, LOCK_FILE, EN_RUN, EN_RESET, check_other_motor_processes


def reset_enable(motor):
    if type(motor) is not int or motor not in MOTORS:
        raise ValueError('请选择 1 号或 2 号电机')
    check_other_motor_processes()
    step_pin, _, en_pin = MOTORS[motor]
    step = enable = None
    with open(LOCK_FILE, 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            step = Output(step_pin, 0)
            enable = Output(en_pin, EN_RESET)  # EN cycle only; alarm clearance requires external observation.
            print(f'电机{motor}: STEP GPIO{step_pin}=低；EN GPIO{en_pin}=高（禁用电平），等待 1 秒', flush=True)
            time.sleep(1.0)
            enable.write(EN_RUN)
            time.sleep(0.2)
        finally:
            try:
                if step is not None:
                    step.write(0)
            finally:
                try:
                    if enable is not None:
                        enable.write(EN_RUN)
                finally:
                    for output in (step, enable):
                        if output is not None:
                            output.close()
    print(f'电机{motor}: EN 已恢复低电平（运行电平）。未发送运动脉冲；请观察报警灯，未接入报警反馈。', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motor', type=int, choices=(1, 2), required=True)
    args = parser.parse_args()
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupt)
    reset_enable(args.motor)
