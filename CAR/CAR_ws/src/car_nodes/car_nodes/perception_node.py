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
  body_filter_distance   (float, 0.25)  车身自遮罩半径 m，小于该距离的回波丢弃
  min_cluster_points     (int,   3)     聚类最少 bin 数，少于此进入细目标候选
  thin_persist_frames    (int,   3)     细目标（椅腿等）连续命中帧数确认阈值
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
        # 实机排障：车身结构在雷达尾线方向产生大量 0.2m 左右的零散回波，
        # 聚类后按距离截断时会把所有真实障碍挤出 max_obstacles 列表
        self.declare_parameter('body_filter_distance', 0.25)  # 车身自遮罩半径 m
        self.declare_parameter('min_cluster_points', 3)       # 少于此点数视为噪点
        # 细目标（椅腿/桌腿只占 1~2 个 bin）：连续 N 帧命中同一角度才采纳，
        # 兼顾细目标识别与随机噪点抑制（10Hz 下 N=3 ≈ 0.3s 确认）
        self.declare_parameter('thin_persist_frames', 3)

        self.max_obstacles = self.get_parameter('max_obstacles').value
        publish_rate = self.get_parameter('publish_rate').value
        self.enable_vision = self.get_parameter('enable_vision').value
        self.vision_default_distance = \
            self.get_parameter('vision_default_distance').value
        self.vision_diff_threshold = self.get_parameter('vision_diff_threshold').value
        self.body_filter_distance = \
            self.get_parameter('body_filter_distance').value
        self.min_cluster_points = \
            int(self.get_parameter('min_cluster_points').value)
        self.thin_persist_frames = \
            int(self.get_parameter('thin_persist_frames').value)

        self.latest_scan = None
        self.vision_obstacle = None  # 视觉估计的障碍（单条）
        self._thin_hits = {}  # 细目标持续性：2° 角度格 -> [连续命中帧数, 簇]

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
        """角度分 bin 聚类，每个聚类输出一个障碍（source=0）。

        N10P 双回波交错：同方向相邻点距离在远近回波间跳变（如 1.5m/4.7m），
        按原始点序聚类会被全部打成单点簇。改为每 1° 一个 bin 取最近回波
        （避障只关心最近距离），再按 bin 间距离连续性聚类。
        """
        obstacles = []
        if scan is None or len(scan.ranges) == 0:
            return obstacles

        ranges = np.asarray(scan.ranges, dtype=np.float32)
        n = len(ranges)
        # 车身自遮罩：小于 body_filter_distance 的回波视为车体自身，直接丢弃
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) \
            & (ranges <= scan.range_max) \
            & (ranges >= self.body_filter_distance)
        if not valid.any():
            return obstacles

        bin_count = 360
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        bin_idx = np.floor(
            (angles + math.pi) / (2.0 * math.pi) * bin_count).astype(np.int64)
        bin_idx = np.clip(bin_idx, 0, bin_count - 1)
        bin_min = np.full(bin_count, np.inf, dtype=np.float32)
        np.minimum.at(bin_min, bin_idx[valid], ranges[valid])

        cluster_gap = 0.3  # m，相邻 bin 最近距离差小于阈值认为同一障碍
        cluster = []       # [(bin_center_angle, min_dist), ...]
        thin = []          # 细目标候选簇（bin 数不足 min_cluster_points）
        for b in range(bin_count):
            dist = bin_min[b]
            if np.isinf(dist):
                if cluster:
                    self._flush_cluster(obstacles, thin, cluster)
                    cluster = []
                continue
            if cluster and abs(dist - cluster[-1][1]) > cluster_gap:
                self._flush_cluster(obstacles, thin, cluster)
                cluster = []
            angle = -math.pi + (b + 0.5) * (2.0 * math.pi / bin_count)
            cluster.append((angle, float(dist)))
        if cluster:
            self._flush_cluster(obstacles, thin, cluster)

        # 细目标持续性确认：同一 2° 角度格连续命中 N 帧才采纳为障碍
        hits = {}
        for cluster in thin:
            center_deg = math.degrees(cluster[len(cluster) // 2][0])
            key = int(round(center_deg / 2.0))
            prev = self._thin_hits.get(key)
            count = (prev[0] if prev else 0) + 1
            hits[key] = [count, cluster]
            if count >= self.thin_persist_frames:
                obstacles.append(self._make_obstacle(cluster))
        self._thin_hits = hits
        return obstacles

    def _flush_cluster(self, obstacles, thin, cluster):
        """正常簇转障碍；bin 数过少的簇进入细目标候选（椅腿/桌腿等）"""
        if len(cluster) < self.min_cluster_points:
            thin.append(cluster)
            return
        obstacles.append(self._make_obstacle(cluster))

    @staticmethod
    def _make_obstacle(cluster):
        obs = Obstacle()
        dists = [c[1] for c in cluster]
        obs.distance = float(min(dists))
        obs.angle = float(cluster[len(cluster) // 2][0])  # 簇中心角度
        # 簇宽（弧长）的一半作为等效半径（每 bin 1°）
        width = obs.distance * len(cluster) * math.radians(1.0)
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
