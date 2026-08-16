"""k210_camera_driver.extract_jpeg_frames 的纯函数单测（无需 ROS 环境）"""

from car_nodes.k210_camera_driver import extract_jpeg_frames

SOI = b'\xff\xd8'
EOI = b'\xff\xd9'
FRAME_A = SOI + b'\xaa\x01\x02' + EOI
FRAME_B = SOI + b'\xbb\x03\x04\x05' + EOI


def test_single_complete_frame():
    buf = bytearray(FRAME_A)
    frames = extract_jpeg_frames(buf)
    assert frames == [FRAME_A]
    assert bytes(buf) == b''


def test_frame_split_across_feeds():
    buf = bytearray()
    assert extract_jpeg_frames(buf) == []
    buf.extend(FRAME_A[:3])
    assert extract_jpeg_frames(buf) == []  # 不完整帧保留等待
    assert bytes(buf) == FRAME_A[:3]
    buf.extend(FRAME_A[3:])
    assert extract_jpeg_frames(buf) == [FRAME_A]
    assert bytes(buf) == b''


def test_noise_before_and_between_frames():
    buf = bytearray(b'\x00\x11\x22' + FRAME_A + b'\x99' + FRAME_B + b'\x77')
    frames = extract_jpeg_frames(buf)
    assert frames == [FRAME_A, FRAME_B]
    assert bytes(buf) == b'\x77' or bytes(buf) == b''  # 尾部噪声在下次调用时清除
    # 尾部噪声（无 SOI）在下一次切帧时被清空
    assert extract_jpeg_frames(buf) == []
    assert bytes(buf) == b''


def test_pure_noise_cleared():
    buf = bytearray(b'\x01\x02\x03')
    assert extract_jpeg_frames(buf) == []
    assert bytes(buf) == b''


def test_multiple_frames_one_feed():
    buf = bytearray(FRAME_A + FRAME_B + FRAME_A)
    frames = extract_jpeg_frames(buf)
    assert frames == [FRAME_A, FRAME_B, FRAME_A]
    assert bytes(buf) == b''


def test_truncated_tail_kept():
    buf = bytearray(FRAME_A + FRAME_B[:4])
    frames = extract_jpeg_frames(buf)
    assert frames == [FRAME_A]
    assert bytes(buf) == FRAME_B[:4]
