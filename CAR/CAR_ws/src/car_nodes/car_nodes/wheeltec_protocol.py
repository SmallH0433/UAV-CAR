"""WHEELTEC 运动底盘串口协议编解码（纯 Python，无 ROS 依赖）。

协议来源：厂商资料《串口通信控制与反馈_2026-8-12.pdf》
（资料包：1.WHEELTEC ROS机器人通用资料/2.运动底盘的控制与运动学解析/5.串口控制讲解）。
ROS 教育机器人与大型 ROS 科研机器人（含 R680）使用同一协议，串口 3，波特率 115200。

下行帧（上位机 → STM32，11 字节）：
  [0]    0x7B 帧头
  [1:3]  预留 0x00 0x00
  [3:5]  X 目标速度 short 大端，单位 mm/s
  [5:7]  Y 目标速度 short 大端，单位 mm/s（仅全向车型有效，差速车填 0）
  [7:9]  Z 目标速度 short 大端，rad/s × 1000
  [9]    BCC = 前 9 字节异或
  [10]   0x7D 帧尾

上行帧（STM32 → 上位机，24 字节）：
  [0]     0x7B 帧头
  [1]     flag_stop（0x00=电机使能，其他=失能）
  [2:4]   X 速度 short 大端 mm/s
  [4:6]   Y 速度 short 大端 mm/s
  [6:8]   Z 速度 short 大端 rad/s × 1000
  [8:14]  XYZ 加速度计原始数据（/1672 → m/s²）
  [14:20] XYZ 陀螺仪原始数据（/3753 → rad/s）
  [20:22] 电池电压 short 大端 mV
  [22]    BCC = 前 22 字节异或
  [23]    0x7D 帧尾

方向约定：厂商文档 3.1 节示例中 Z 轴速度负值对应顺时针旋转，
即正值为逆时针，与 ROS REP-103 一致，收发均无需换号。
"""

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
DOWNLINK_LEN = 11
UPLINK_LEN = 24
ACCEL_SCALE = 1672.0   # 加速度计原始数据 → m/s²
GYRO_SCALE = 3753.0    # 陀螺仪原始数据 → rad/s


def bcc(data):
    """BCC 校验：全部字节异或。"""
    x = 0
    for b in data:
        x ^= b
    return x


def _to_int16(value):
    return max(-32768, min(32767, int(round(value))))


def build_downlink_frame(vx_m_s, vy_m_s, vz_rad_s):
    """(vx m/s, vy m/s, vz rad/s) → 11 字节下行控制帧。"""
    vx = _to_int16(vx_m_s * 1000.0)
    vy = _to_int16(vy_m_s * 1000.0)
    vz = _to_int16(vz_rad_s * 1000.0)
    frame = bytearray(DOWNLINK_LEN)
    frame[0] = FRAME_HEADER
    frame[3:5] = vx.to_bytes(2, 'big', signed=True)
    frame[5:7] = vy.to_bytes(2, 'big', signed=True)
    frame[7:9] = vz.to_bytes(2, 'big', signed=True)
    frame[9] = bcc(frame[:9])
    frame[10] = FRAME_TAIL
    return bytes(frame)


def parse_uplink_frame(frame):
    """解析 24 字节上行帧 → dict；帧头/帧尾/校验错误返回 None。"""
    if len(frame) != UPLINK_LEN:
        return None
    if frame[0] != FRAME_HEADER or frame[UPLINK_LEN - 1] != FRAME_TAIL:
        return None
    if bcc(frame[:UPLINK_LEN - 2]) != frame[UPLINK_LEN - 2]:
        return None

    def s(i):
        return int.from_bytes(frame[i:i + 2], 'big', signed=True)

    return {
        'flag_stop': frame[1],                    # 0x00=电机使能
        'vx': s(2) / 1000.0,                      # m/s
        'vy': s(4) / 1000.0,                      # m/s
        'vz': s(6) / 1000.0,                      # rad/s
        'accel': (s(8) / ACCEL_SCALE, s(10) / ACCEL_SCALE,
                  s(12) / ACCEL_SCALE),           # m/s²
        'gyro': (s(14) / GYRO_SCALE, s(16) / GYRO_SCALE,
                 s(18) / GYRO_SCALE),             # rad/s
        'voltage': s(20) / 1000.0,                # V
    }


class UplinkFrameParser:
    """流式上行帧解析器：容忍串口字节流中的噪声、断帧与粘包。"""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data):
        """喂入新收到的字节，返回本次解析出的完整帧列表（按到达顺序）。"""
        if data:
            self._buf += data
        frames = []
        while True:
            idx = self._buf.find(bytes([FRAME_HEADER]))
            if idx < 0:
                self._buf.clear()
                break
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < UPLINK_LEN:
                break
            parsed = parse_uplink_frame(bytes(self._buf[:UPLINK_LEN]))
            if parsed is not None:
                frames.append(parsed)
                del self._buf[:UPLINK_LEN]
            else:
                del self._buf[0]  # 假帧头，滑动一个字节重新找
        return frames
