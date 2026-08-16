"""K210 前视摄像头驱动节点（MaixPy JPEG 串口推流，固件见 scripts/k210_firmware.py）

发布契约与 camera_driver 相同，可直接替换前摄：
  /camera/image_raw   (sensor_msgs/Image, rgb8)  frame_id=camera_optical_frame
  /camera/camera_info (sensor_msgs/CameraInfo)
参数：
  port       (str,  '/dev/ttyUSB0')        K210 USB 串口设备
  baud       (int,  921600)                波特率，必须与固件 BAUD 一致
  image_topic(str,  '/camera/image_raw')   图像话题
  info_topic (str,  '/camera/camera_info') CameraInfo 话题
  frame_id   (str,  'camera_optical_frame') 图像与 CameraInfo 的 frame_id

K210 经 USB 线直连树莓派（同时供电），固件连续输出 JPEG 帧
（FFD8 开始、FFD9 结束），本节点按标记切帧、cv2 解码后发布。
"""

import threading

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

JPEG_SOI = b'\xff\xd8'
JPEG_EOI = b'\xff\xd9'
MAX_BUFFER = 4 * 1024 * 1024  # 接收缓冲上限，超过说明流已损坏，清空重新对齐


def extract_jpeg_frames(buf):
    """从接收缓冲（bytearray）切出完整 JPEG 帧，残余数据留在 buf 中。

    纯函数式切帧（仅改动传入的 buf），可独立单测：
    - 返回所有完整帧（FFD8...FFD9）的 bytes 列表；
    - 帧头前的噪声、帧间噪声丢弃；不完整帧保留在 buf 等待后续数据。
    """
    frames = []
    while True:
        start = buf.find(JPEG_SOI)
        if start < 0:  # 无帧头：整段都是噪声
            buf.clear()
            break
        if start > 0:  # 丢弃帧头前的噪声
            del buf[:start]
        end = buf.find(JPEG_EOI, 2)
        if end < 0:  # 帧不完整，等更多数据
            break
        frames.append(bytes(buf[:end + 2]))
        del buf[:end + 2]
    return frames


class K210CameraDriverNode(Node):
    def __init__(self):
        super().__init__('k210_camera_driver_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('info_topic', '/camera/camera_info')
        self.declare_parameter('frame_id', 'camera_optical_frame')

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub_image = self.create_publisher(
            Image, self.get_parameter('image_topic').value, 10)
        self.pub_info = self.create_publisher(
            CameraInfo, self.get_parameter('info_topic').value, 10)

        self._serial = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f'K210 摄像头：{self.port} @ {self.baud}，等待 JPEG 流...')

    def _open_serial(self):
        import serial  # pyserial
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0.5)
            self.get_logger().info(f'已打开串口 {self.port}')
            return True
        except Exception as exc:
            self.get_logger().warn(
                f'无法打开串口 {self.port}：{exc}', throttle_duration_sec=5.0)
            self._serial = None
            return False

    def _reader_loop(self):
        buf = bytearray()
        while not self._stop.is_set():
            if self._serial is None and not self._open_serial():
                self._stop.wait(2.0)  # 串口未就绪（未接设备/规则未配），2s 重试
                continue
            try:
                data = self._serial.read(4096)
            except Exception as exc:  # 设备拔出等：关闭后重连
                self.get_logger().warn(f'串口读取失败：{exc}，2s 后重连')
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
                self._stop.wait(2.0)
                continue
            if not data:
                continue
            buf.extend(data)
            if len(buf) > MAX_BUFFER:
                self.get_logger().warn('接收缓冲溢出，清空重新对齐帧头')
                buf.clear()
                continue
            for frame in extract_jpeg_frames(buf):
                self._publish_frame(frame)

    def _publish_frame(self, jpeg_bytes):
        import cv2
        frame = cv2.imdecode(
            np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('JPEG 解码失败，丢弃一帧',
                                   throttle_duration_sec=5.0)
            return
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # 与 camera_driver 同约定
        height, width = frame.shape[:2]

        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = height
        msg.width = width
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = frame.tobytes()
        self.pub_image.publish(msg)
        self.pub_info.publish(self._make_camera_info(stamp, width, height))

    def _make_camera_info(self, stamp, width, height):
        """简单针孔模型填 CameraInfo（与 camera_driver 同款估计）"""
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = width
        info.height = height
        fx = fy = 0.5 * width  # 简单估计：视场角约 90°
        cx = width / 2.0
        cy = height / 2.0
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.distortion_model = 'plumb_bob'
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def destroy_node(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = K210CameraDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
