"""相机驱动节点

默认发布（前视）：
  /camera/image_raw   (sensor_msgs/Image)      frame_id=camera_optical_frame
  /camera/camera_info (sensor_msgs/CameraInfo)
参数：
  device     (str,  '/dev/video0')           摄像头设备路径
  width      (int,  640)                     图像宽度
  height     (int,  480)                     图像高度
  fps        (int,  15)                      发布帧率
  simulate   (bool, True)                    True=生成渐变测试图；False=cv2 读真实摄像头
  image_topic(str,  '/camera/image_raw')     图像话题
  info_topic (str,  '/camera/camera_info')   CameraInfo 话题
  frame_id   (str,  'camera_optical_frame')  图像与 CameraInfo 的 frame_id

后置摄像头（USB，如 /dev/video1）以第二实例运行：
  ros2 run car_nodes camera_driver_node --ros-args \
    -p device:=/dev/video1 -p simulate:=false \
    -p image_topic:=/camera/rear/image_raw \
    -p info_topic:=/camera/rear/camera_info \
    -p frame_id:=rear_camera_optical_frame
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CameraDriverNode(Node):
    def __init__(self):
        super().__init__('camera_driver_node')
        # 声明参数
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15)
        self.declare_parameter('simulate', True)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('info_topic', '/camera/camera_info')
        self.declare_parameter('frame_id', 'camera_optical_frame')

        self.device = self.get_parameter('device').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.simulate = self.get_parameter('simulate').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub_image = self.create_publisher(
            Image, self.get_parameter('image_topic').value, 10)
        self.pub_info = self.create_publisher(
            CameraInfo, self.get_parameter('info_topic').value, 10)

        self.cap = None
        if not self.simulate:
            # 仅在真实模式下导入 cv2，避免仿真环境缺依赖报错
            import cv2
            self.cap = cv2.VideoCapture(self.device)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if not self.cap.isOpened():
                self.get_logger().error(f'无法打开摄像头 {self.device}')
            else:
                self.get_logger().info(f'已打开摄像头 {self.device}')
        else:
            self.get_logger().info('仿真模式：发布渐变测试图')

        self.timer = self.create_timer(1.0 / fps, self.timer_cb)

    def timer_cb(self):
        stamp = self.get_clock().now().to_msg()
        if self.simulate:
            frame = self._make_test_image()
        else:
            if self.cap is None or not self.cap.isOpened():
                return
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn('读取摄像头帧失败', throttle_duration_sec=2.0)
                return
            # OpenCV 默认 BGR，转成 RGB 发布
            import cv2
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = self.width * 3
        msg.data = frame.tobytes()
        self.pub_image.publish(msg)
        self.pub_info.publish(self._make_camera_info(stamp))

    def _make_test_image(self):
        """生成 RGB 渐变测试图（不依赖 OpenCV）"""
        x = np.linspace(0, 255, self.width, dtype=np.uint8)
        y = np.linspace(0, 255, self.height, dtype=np.uint8)
        xx, yy = np.meshgrid(x, y)
        frame = np.stack([xx, yy, 255 - xx], axis=-1).astype(np.uint8)
        return frame

    def _make_camera_info(self, stamp):
        """按 640x480 简单针孔模型填 CameraInfo"""
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        fx = fy = 0.5 * self.width  # 简单估计：视场角约 90°
        cx = self.width / 2.0
        cy = self.height / 2.0
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.distortion_model = 'plumb_bob'
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
