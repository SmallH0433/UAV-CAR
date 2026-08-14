"""感知节点

订阅：
  /scan             (sensor_msgs/LaserScan)
  /camera/image_raw (sensor_msgs/Image)
发布：
  /perception/obstacles (car_interfaces/ObstacleArray)
参数：
  max_obstacles          (int,   16)    最多保留的障碍数（按距离排序）
  publish_rate           (float, 10.0)  发布频率 Hz
  enable_vision          (bool,  True)  是否启用视觉辅助判断
  vision_default_distance(float, 1.5)   视觉命中时估计的障碍距离 m
  vision_diff_threshold  (float, 40.0)  左右半区亮度差阈值（0-255）
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan

from car_interfaces.msg import Obstacle, ObstacleArray


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        # 声明参数
        self.declare_parameter('max_obstacles', 16)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('enable_vision', True)
        self.declare_parameter('vision_default_distance', 1.5)
        self.declare_parameter('vision_diff_threshold', 40.0)

        self.max_obstacles = self.get_parameter('max_obstacles').value
        publish_rate = self.get_parameter('publish_rate').value
        self.enable_vision = self.get_parameter('enable_vision').value
        self.vision_default_distance = \
            self.get_parameter('vision_default_distance').value
        self.vision_diff_threshold = self.get_parameter('vision_diff_threshold').value

        self.latest_scan = None
        self.vision_obstacle = None  # 视觉估计的障碍（单条）

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        if self.enable_vision:
            self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)

        self.pub_obstacles = self.create_publisher(
            ObstacleArray, '/perception/obstacles', 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_cb)

    # ---------- 雷达处理 ----------
    def scan_cb(self, msg):
        self.latest_scan = msg

    def _lidar_obstacles(self, scan):
        """按角度连续性聚类，每个聚类输出一个障碍（source=0）"""
        obstacles = []
        if scan is None or len(scan.ranges) == 0:
            return obstacles

        ranges = np.asarray(scan.ranges, dtype=np.float32)
        n = len(ranges)
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) \
            & (ranges <= scan.range_max)

        # 相邻有效点距离差小于阈值认为属于同一障碍
        cluster_gap = 0.3  # m
        cluster = []
        for i in range(n):
            if valid[i]:
                if cluster:
                    prev = cluster[-1]
                    if abs(ranges[i] - ranges[prev]) > cluster_gap:
                        obstacles.append(self._cluster_to_obstacle(scan, ranges, cluster))
                        cluster = []
                cluster.append(i)
            else:
                if cluster:
                    obstacles.append(self._cluster_to_obstacle(scan, ranges, cluster))
                    cluster = []
        if cluster:
            obstacles.append(self._cluster_to_obstacle(scan, ranges, cluster))
        return obstacles

    def _cluster_to_obstacle(self, scan, ranges, cluster):
        """把一个点簇转换为障碍：最近点距离、簇中心角度、簇宽等效半径"""
        obs = Obstacle()
        dists = ranges[cluster]
        min_idx = cluster[int(np.argmin(dists))]
        distance = float(ranges[min_idx])
        # 簇中心角度
        mid = cluster[len(cluster) // 2]
        angle = scan.angle_min + mid * scan.angle_increment
        # 簇宽（弧长）的一半作为等效半径
        width = distance * len(cluster) * scan.angle_increment
        obs.angle = float(angle)
        obs.distance = distance
        obs.radius = float(max(width / 2.0, 0.05))
        obs.velocity = 0.0
        obs.source = 0
        return obs

    # ---------- 视觉处理（轻量占位实现） ----------
    def image_cb(self, msg):
        """下半区左右亮度差超阈值 → 认为前方地面有疑似障碍（source=1）"""
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
        except ValueError:
            return
        gray = img.mean(axis=2)
        lower = gray[msg.height // 2:, :]  # 只看下半区（地面）
        left_mean = float(lower[:, :msg.width // 2].mean())
        right_mean = float(lower[:, msg.width // 2:].mean())
        if abs(left_mean - right_mean) > self.vision_diff_threshold:
            obs = Obstacle()
            obs.angle = 0.0
            obs.distance = self.vision_default_distance
            obs.radius = 0.2
            obs.velocity = 0.0
            obs.source = 1
            self.vision_obstacle = obs
        else:
            self.vision_obstacle = None

    # ---------- 定时发布 ----------
    def timer_cb(self):
        obstacles = self._lidar_obstacles(self.latest_scan)
        if self.enable_vision and self.vision_obstacle is not None:
            obstacles.append(self.vision_obstacle)
        # 按距离排序，保留最近 N 个
        obstacles.sort(key=lambda o: o.distance)
        obstacles = obstacles[:self.max_obstacles]

        out = ObstacleArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.obstacles = obstacles
        self.pub_obstacles.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
