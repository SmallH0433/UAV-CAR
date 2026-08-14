"""WHEELTEC 串口协议编解码单测（纯函数，不需要 ROS 图）。

测试向量取自厂商文档《串口通信控制与反馈》3.1 节的收发示例。
"""

import pytest

from car_nodes.wheeltec_protocol import (
    DOWNLINK_LEN, UPLINK_LEN, UplinkFrameParser, build_downlink_frame,
    parse_uplink_frame)

# 厂商文档 3.1 ③ 发送示例：X 目标速度 100mm/s
DOC_DOWNLINK_100MM = bytes(
    [0x7B, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x7D])

# 厂商文档 3.1 ② 接收示例：vx=155mm/s, vz=-0.033rad/s, az≈9.9m/s², 23.431V
DOC_UPLINK = bytes([
    0x7B, 0x00, 0x00, 0x9B, 0x00, 0x00, 0xFF, 0xDF,
    0x00, 0x60, 0x00, 0x0C, 0x40, 0xA8, 0xFF, 0xFD,
    0x00, 0x06, 0x00, 0x1E, 0x5B, 0x87, 0x82, 0x7D,
])


def test_downlink_matches_vendor_example():
    frame = build_downlink_frame(0.1, 0.0, 0.0)
    assert len(frame) == DOWNLINK_LEN
    assert frame == DOC_DOWNLINK_100MM


def test_downlink_negative_uses_twos_complement():
    # 文档：-100mm/s 的补码为 0xFF9C
    frame = build_downlink_frame(-0.1, 0.0, 0.0)
    assert frame[3:5] == b'\xff\x9c'


def test_downlink_omni_y_and_rotation():
    frame = build_downlink_frame(0.0, 0.05, -0.5)
    assert frame[5:7] == b'\x00\x32'   # 50 mm/s
    assert frame[7:9] == (-500).to_bytes(2, 'big', signed=True)


def test_downlink_clamps_to_int16():
    frame = build_downlink_frame(100.0, 0.0, -100.0)
    assert frame[3:5] == (32767).to_bytes(2, 'big', signed=True)
    assert frame[7:9] == (-32768).to_bytes(2, 'big', signed=True)


def test_uplink_matches_vendor_example():
    parsed = parse_uplink_frame(DOC_UPLINK)
    assert parsed is not None
    assert parsed['flag_stop'] == 0x00
    assert parsed['vx'] == pytest.approx(0.155)
    assert parsed['vy'] == pytest.approx(0.0)
    assert parsed['vz'] == pytest.approx(-0.033)
    assert parsed['accel'][2] == pytest.approx(9.8995, abs=1e-3)
    assert parsed['voltage'] == pytest.approx(23.431)


def test_uplink_rejects_bad_bcc_and_tail():
    bad_bcc = bytearray(DOC_UPLINK)
    bad_bcc[3] ^= 0x01
    assert parse_uplink_frame(bytes(bad_bcc)) is None
    bad_tail = bytearray(DOC_UPLINK)
    bad_tail[UPLINK_LEN - 1] = 0x00
    assert parse_uplink_frame(bytes(bad_tail)) is None


def test_parser_resyncs_after_noise():
    parser = UplinkFrameParser()
    frames = parser.feed(b'\x00\x11\x7B\xaa' + DOC_UPLINK)
    assert len(frames) == 1
    assert frames[0]['voltage'] == pytest.approx(23.431)


def test_parser_handles_split_and_sticky_frames():
    parser = UplinkFrameParser()
    # 断帧：前半段不出帧，后半段补齐后出帧
    assert parser.feed(DOC_UPLINK[:10]) == []
    frames = parser.feed(DOC_UPLINK[10:] + DOC_UPLINK)
    # 粘包：拼在后面的第二帧也一并解出
    assert len(frames) == 2
