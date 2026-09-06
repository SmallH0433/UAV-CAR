"""ESP8266 双 T8 双向丝杆驱动的文本行协议编解码（纯 Python，可单测）。

固件工程：tools/leadscrew42（仓库 esp8266-deploy 分支），串口 115200 8N1。
下行：每行一条文本命令；上行：OK/ERR 应答、DONE 到位上报、
POS 查询返回的 G# 状态行与 SPEED 行，以及上电 banner（非协议行，忽略）。
"""

import re

# 与 car_interfaces/LeadscrewCommand 的 CMD_* 常量一致
CMD_STOP, CMD_IN, CMD_OUT, CMD_RELAX, CMD_LOCK = range(5)

# 与 car_interfaces/LeadscrewStatus 的 STATE_* 常量一致
STATE_NAMES = ['AT_OUTER', 'MOVING_IN', 'AT_INNER', 'MOVING_OUT', 'HOLD_MID']
STATE_IDS = {name: i for i, name in enumerate(STATE_NAMES)}

_RE_DONE = re.compile(r'^DONE G([12]) (IN|OUT) pos=(-?[\d.]+)mm$')
_RE_STATUS = re.compile(
    r'^G([12]) state=(\w+) pos=(-?[\d.]+)mm steps=(-?\d+) en=([01])$')
_RE_SPEED = re.compile(r'^SPEED (\d+) steps/s')

_CMD_VERB = {CMD_IN: 'IN', CMD_OUT: 'OUT', CMD_RELAX: 'RELAX', CMD_LOCK: 'LOCK'}


def build_command(group, command, speed=0):
    """ROS 指令 → 固件文本命令（bytes）。

    group: 0=两组同时 1=组1 2=组2；command: CMD_*；speed: 0=不改变。
    STOP 是全局命令（固件语义），忽略 group。
    """
    lines = []
    if speed:
        lines.append(f'SPEED {int(speed)}')
    if command == CMD_STOP:
        lines.append('STOP')
    else:
        verb = _CMD_VERB[command]
        lines.append(verb if group == 0 else f'{verb} {group}')
    return ('\n'.join(lines) + '\n').encode('ascii')


def parse_line(line):
    """解析一行固件输出，返回 dict；非协议行（banner/噪声）返回 None。

    返回 type:
      status: {group, state, state_id, pos_mm, steps, enabled}
      done:   {group, verb, pos_mm}
      speed:  {steps_per_sec}
      ok/err: {text}
    """
    line = line.strip()
    m = _RE_STATUS.match(line)
    if m:
        state = m.group(2)
        return {'type': 'status', 'group': int(m.group(1)), 'state': state,
                'state_id': STATE_IDS.get(state, 0),
                'pos_mm': float(m.group(3)), 'steps': int(m.group(4)),
                'enabled': m.group(5) == '1'}
    m = _RE_DONE.match(line)
    if m:
        return {'type': 'done', 'group': int(m.group(1)),
                'verb': m.group(2), 'pos_mm': float(m.group(3))}
    m = _RE_SPEED.match(line)
    if m:
        return {'type': 'speed', 'steps_per_sec': int(m.group(1))}
    if line.startswith('OK'):
        return {'type': 'ok', 'text': line}
    if line.startswith('ERR'):
        return {'type': 'err', 'text': line}
    return None
