"""Minimal UGV web gateway: static teleop page, JSON status API, camera JPEG.

Trimmed from the air_ground_sim dashboard gateway to the UGV teleop subset:
no missions, no UAV, no audit/auth machinery. Binds to localhost by
default and exposes:

  GET  /                 static teleop page (car_sim/web/index.html)
  GET  /api/health       liveness probe
  GET  /api/status       mux/gateway status + /odom pose + camera state + /fix GPS
  GET  /api/scan.json    latest /scan downsampled to polar points for the lidar canvas
  POST /api/ugv/teleop   {"linear": m/s, "angular": rad/s} operator command
  POST /api/mapping      {"enable": bool, "auto_cruise": bool} 一键建图启停
  POST /api/photo        保存当前前摄帧到 ~/photos/photo_<时间戳>.jpg（已保留）
  GET  /api/photo/download  最新前摄帧 JPEG，带 Content-Disposition 供浏览器直接下载
  GET  /api/camera.jpg   latest gz-bridge front camera frame as JPEG (503 if absent)
  GET  /api/camera_rear.jpg  latest gz-bridge rear camera frame as JPEG (503 if absent)

Teleop is fail-closed: commands older than ``teleop_watchdog_s`` are replaced
by a zero command, and the operator heartbeat published alongside teleop is
what lets ugv_control_mux grant the operator authority over the autonomy
(avoidance) command input.
"""

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import signal
import subprocess
import threading
import time
from collections import deque
from datetime import datetime
from typing import Optional, Tuple
from urllib.parse import urlparse

import yaml
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, LaserScan, NavSatFix
from std_msgs.msg import Bool, String

import tf2_ros
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from .runtime_timing import create_steady_timer

try:  # Camera JPEG support is optional; the rest of the gateway works without.
    import cv2
    from cv_bridge import CvBridge

    _CAMERA_AVAILABLE = True
except ImportError:
    cv2 = None
    CvBridge = None
    _CAMERA_AVAILABLE = False


def clamped_teleop(
    payload: dict, max_linear: float, max_angular: float
) -> Tuple[float, float]:
    """Validate a teleop JSON payload and clamp it to the motion envelope."""

    if not isinstance(payload, dict):
        raise ValueError("json_object_required")
    linear = float(payload.get("linear", 0.0))
    angular = float(payload.get("angular", 0.0))
    if not (math.isfinite(linear) and math.isfinite(angular)):
        raise ValueError("non_finite_command")
    return (
        max(-max_linear, min(max_linear, linear)),
        max(-max_angular, min(max_angular, angular)),
    )


def _json_message(message: String) -> dict:
    try:
        value = json.loads(message.data)
        return value if isinstance(value, dict) else {}
    except json.JSONDecodeError:
        return {}


def save_photo(jpeg_bytes: bytes, photos_dir: str) -> str:
    """把一帧 JPEG 写入 photos_dir/photo_<时间戳>.jpg，返回文件路径。

    同一秒内连拍时追加 _1/_2… 序号避免覆盖。
    """
    os.makedirs(photos_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(photos_dir, "photo_" + stamp + ".jpg")
    seq = 0
    while os.path.exists(path):
        seq += 1
        path = os.path.join(photos_dir, f"photo_{stamp}_{seq}.jpg")
    with open(path, "wb") as fh:
        fh.write(jpeg_bytes)
    return path


def _yaw_from_quaternion(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def _index_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        candidate = os.path.join(
            get_package_share_directory("car_sim"), "web", "index.html"
        )
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "web", "index.html")


class WebGateway(Node):
    """HTTP front-end for UGV teleop and status monitoring."""

    def __init__(self) -> None:
        super().__init__("web_gateway")
        self.declare_parameter("bind_address", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("teleop_topic", "/ugv/teleop/cmd_vel")
        self.declare_parameter("heartbeat_topic", "/ugv/operator/heartbeat")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("rear_camera_topic", "/camera/rear/image_raw")
        self.declare_parameter("teleop_watchdog_s", 0.35)
        self.declare_parameter("max_linear_mps", 0.5)
        self.declare_parameter("max_angular_rps", 0.7)
        self.declare_parameter("camera_max_rate_hz", 5.0)
        self.declare_parameter("camera_stale_after_s", 3.0)
        self.declare_parameter("jpeg_quality", 72)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_max_points", 720)
        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("mapping_lidar_x", 0.1)
        self.declare_parameter("mapping_lidar_y", 0.0)
        self.declare_parameter("mapping_lidar_z", 0.15)
        self.declare_parameter("avoidance_node_name", "/avoidance_node")
        self.declare_parameter("mux_node_name", "/ugv_control_mux")
        # 自主导航后端：avoidance=自研避障节点（/goal_pose 话题，odom 系）；
        # nav2=AMCL+Nav2（NavigateToPose action，map 系）
        self.declare_parameter("nav_backend", "avoidance")
        # 网页导航/巡航加载的地图名（~/maps/<name>.pgm/.yaml）
        self.declare_parameter("nav_map_name", "map_20260820_193257")

        self.bind_address = str(self.get_parameter("bind_address").value)
        self.port = int(self.get_parameter("port").value)
        self.max_linear = float(self.get_parameter("max_linear_mps").value)
        self.max_angular = float(self.get_parameter("max_angular_rps").value)
        self.teleop_watchdog = max(
            0.1, float(self.get_parameter("teleop_watchdog_s").value)
        )
        self.camera_interval = 1.0 / max(
            0.2, float(self.get_parameter("camera_max_rate_hz").value)
        )
        self.camera_stale_after = max(
            self.camera_interval * 2.0,
            float(self.get_parameter("camera_stale_after_s").value),
        )
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.nav_backend = str(self.get_parameter("nav_backend").value)
        self.nav_map_name = str(self.get_parameter("nav_map_name").value)

        self.lock = threading.RLock()
        self.started_at = time.monotonic()
        self.last_teleop_time = 0.0
        self.latest = {"ugv_control_mux": {}, "ugv_command_gateway": {}}
        self.topic_times = {}
        self.pose = None  # [x, y, yaw] in the odom frame
        self.speed_mps = 0.0
        self.image_jpeg: Optional[bytes] = None
        self.image_time = 0.0
        self.rear_image_jpeg: Optional[bytes] = None
        self.rear_image_time = 0.0
        self.bridge = CvBridge() if _CAMERA_AVAILABLE else None
        # 雷达/GPS 网页展示缓存
        self.scan_points = []          # [[angle_deg, dist_m], ...] 降采样极坐标点
        self.scan_range_max = 0.0
        self.scan_time = 0.0
        self.fix = None                # {status, latitude, longitude, altitude}
        self.fix_time = 0.0
        # SLAM 地图缓存（/map OccupancyGrid → PNG）
        self.map_png: Optional[bytes] = None
        self.map_info = None           # {width, height, resolution}
        self.map_time = 0.0
        # 一键建图状态（subprocess 托管 static TF + slam_toolbox）
        self.mapping_active = False
        self.mapping_started = 0.0
        self.mapping_procs = []
        # 自主导航状态
        self.nav_active = False
        self.nav_path = []             # [(x, y), ...] odom 系路径点
        self.nav_goal = None           # (x, y) 目标点
        self.nav_map_data = None       # 当前加载的地图数据（numpy 数组）
        self.nav_map_origin = None     # (x, y, theta) 地图原点
        self.nav_map_resolution = 0.0  # 地图分辨率 m/pixel
        # 自动巡航状态
        self.auto_cruise_active = False
        self.auto_cruise_initial = None  # (x, y) 初始点
        self.auto_cruise_started = 0.0
        self.auto_cruise_returning = False
        # Nav2 导航后端（nav_backend=nav2）：当前 NavigateToPose 目标句柄
        self.nav2_goal_handle = None
        # TF 查询（map→base_footprint，定位模式下网页地图小车位置显示）
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # 默认地图：开机自动加载
        self._load_map(self.nav_map_name)

        self.teleop_publisher = self.create_publisher(
            Twist, str(self.get_parameter("teleop_topic").value), 10
        )
        self.heartbeat_publisher = self.create_publisher(
            Bool, str(self.get_parameter("heartbeat_topic").value), 10
        )
        self.create_subscription(
            String,
            "/ugv/control_mux/status",
            lambda message: self._on_status("ugv_control_mux", message),
            10,
        )
        self.create_subscription(
            String,
            "/ugv/command_gateway/status",
            lambda message: self._on_status("ugv_command_gateway", message),
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix,
            str(self.get_parameter("fix_topic").value),
            self._on_fix,
            qos_profile_sensor_data,
        )
        # 导航路径点到达通知：avoidance_node 到达当前目标点后发布
        self.create_subscription(
            Bool,
            "/nav/goal_reached",
            self._on_nav_goal_reached,
            10,
        )
        # 发布目标点给避障节点
        self.nav_goal_publisher = self.create_publisher(
            PoseStamped, "/goal_pose", 10
        )
        # 取消避障节点的当前目标/巡航（停止导航、停止自动巡航时下发）
        self.nav_cancel_publisher = self.create_publisher(
            Bool, "/nav/cancel", 10
        )
        # Nav2 后端：NavigateToPose action 客户端（nav_backend=nav2 时使用）
        self.nav2_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose")
        # SLAM 地图（slam_toolbox 以 transient_local 发布 /map，订阅需匹配才能
        # 立即收到最近一次地图；未启动 SLAM 时 /api/map.png 返回 503）
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            map_qos,
        )
        if _CAMERA_AVAILABLE:
            self.create_subscription(
                Image,
                str(self.get_parameter("camera_topic").value),
                lambda msg: self._on_image(msg, rear=False),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter("rear_camera_topic").value),
                lambda msg: self._on_image(msg, rear=True),
                qos_profile_sensor_data,
            )
        else:
            self.get_logger().warning(
                "cv2/cv_bridge unavailable: camera JPEG endpoints will return 503"
            )

        self.watchdog_timer = create_steady_timer(self, 0.1, self._teleop_watchdog)

        # 避障节点 enable_cruise 参数客户端（网页巡航开关）
        avoidance = str(self.get_parameter("avoidance_node_name").value)
        self.cruise_set_client = self.create_client(
            SetParameters, f"{avoidance}/set_parameters")
        self.cruise_get_client = self.create_client(
            GetParameters, f"{avoidance}/get_parameters")
        self.cruise_state = None  # True/False；None=避障节点不在线
        self.cruise_poll_timer = create_steady_timer(
            self, 2.0, self._poll_cruise_state)

        # 巡航转向辅助联动：巡航开关状态变化时推送到 mux 的 steering_assist
        mux = str(self.get_parameter("mux_node_name").value)
        self.assist_set_client = self.create_client(
            SetParameters, f"{mux}/set_parameters")
        self._assist_pushed = None  # 已成功推送的 steering_assist 值

        self.http_server = ThreadingHTTPServer(
            (self.bind_address, self.port), self._handler_class()
        )
        self.http_server.daemon_threads = True
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever, name="car-web-gateway", daemon=True
        )
        self.http_thread.start()
        self.get_logger().info(
            f"Web gateway listening on http://{self.bind_address}:{self.port}"
        )

    # ---------------- ROS callbacks ----------------
    def _on_status(self, key: str, message: String) -> None:
        with self.lock:
            self.latest[key] = _json_message(message)
            self.topic_times[key] = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        yaw = _yaw_from_quaternion(message.pose.pose.orientation)
        with self.lock:
            self.pose = [round(position.x, 3), round(position.y, 3), round(yaw, 3)]
            self.speed_mps = round(
                math.hypot(
                    message.twist.twist.linear.x, message.twist.twist.linear.y
                ),
                3,
            )
            self.topic_times["odom"] = time.monotonic()

    def _on_scan(self, message: LaserScan) -> None:
        """降采样 /scan 为 [角度deg, 距离m] 点列，供前端 canvas 极坐标绘图。"""
        count = len(message.ranges)
        if count == 0:
            return
        max_points = max(60, int(self.get_parameter("scan_max_points").value))
        step = max(1, count // max_points)
        points = []
        for i in range(0, count, step):
            distance = message.ranges[i]
            if not math.isfinite(distance) or \
                    distance < message.range_min or distance > message.range_max:
                continue
            angle = message.angle_min + i * message.angle_increment
            points.append([round(math.degrees(angle), 1), round(distance, 2)])
        with self.lock:
            self.scan_points = points
            self.scan_range_max = float(message.range_max)
            self.scan_time = time.monotonic()
            self.topic_times["scan"] = self.scan_time

    def _on_fix(self, message: NavSatFix) -> None:
        # 无定位时 lat/lon/alt 为 NaN：JSON 不支持 NaN 字面量，
        # 浏览器 JSON.parse 会整个失败，必须转成 None
        def finite(value, ndigits):
            return round(value, ndigits) if math.isfinite(value) else None

        with self.lock:
            self.fix = {
                "status": int(message.status.status),
                "latitude": finite(message.latitude, 7),
                "longitude": finite(message.longitude, 7),
                "altitude": finite(message.altitude, 1),
            }
            self.fix_time = time.monotonic()
            self.topic_times["fix"] = self.fix_time

    def _on_map(self, message: OccupancyGrid) -> None:
        """OccupancyGrid → PNG：黑=占据 白=空闲 灰=未知；行序翻转为上北显示。"""
        if cv2 is None:
            return
        import numpy as np

        width, height = message.info.width, message.info.height
        if width == 0 or height == 0:
            return
        data = np.frombuffer(
            bytes(message.data), dtype=np.int8, count=width * height
        ).reshape(height, width)
        img = np.full((height, width), 128, dtype=np.uint8)  # -1 未知=灰
        img[data >= 0] = 255                                  # 空闲=白
        img[data >= 50] = 0                                   # 占据=黑
        img = np.flipud(img)  # 栅格原点在左下，图像原点在左上
        success, encoded = cv2.imencode(".png", img)
        if not success:
            return
        with self.lock:
            self.map_png = encoded.tobytes()
            self.map_info = {
                "width": width,
                "height": height,
                "resolution": round(float(message.info.resolution), 4),
            }
            self.map_time = time.monotonic()
            self.topic_times["map"] = self.map_time

    def _on_nav_goal_reached(self, message: Bool) -> None:
        """避障节点到达当前目标点：发布下一个路径点或结束导航。"""
        if not message.data:
            return
        self._advance_nav_queue()

    def _advance_nav_queue(self) -> None:
        """当前路径点已到达：弹出并发下一个点；全部走完则结束导航。

        avoidance 模式由 /nav/goal_reached 话题触发，nav2 模式由
        NavigateToPose action 成功结果触发。
        """
        with self.lock:
            if not self.nav_active:
                return
            if not self.nav_path:
                # 无本地队列（nav2 单目标直发）：本次导航完成
                self.nav_active = False
                self.nav_goal = None
                self.get_logger().info('自主导航完成')
                return
            self.nav_path.pop(0)  # 移除已到达的路径点
            if not self.nav_path:
                # 所有路径点已到达，导航完成
                self.nav_active = False
                self.nav_goal = None
                self.get_logger().info('自主导航完成')
                return
            next_goal = self.nav_path[0]
        self._publish_nav_goal(next_goal)
        self.get_logger().info(
            f'发布下一个导航目标点：({next_goal[0]:.2f}, {next_goal[1]:.2f})')

    # ---------------- Nav2 导航后端 ----------------
    def _send_nav2_goal(self, goal) -> None:
        """向 Nav2 bt_navigator 发送导航目标（map 系，NavigateToPose action）。"""
        if not self.nav2_client.server_is_ready():
            self.get_logger().error(
                'Nav2 navigate_to_pose action 不可用（nav2 模式未启动？）')
            with self.lock:
                self.nav_active = False
                self.nav_path = []
                self.nav_goal = None
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal[0]
        goal_msg.pose.pose.position.y = goal[1]
        goal_msg.pose.pose.orientation.w = 1.0
        future = self.nav2_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_nav2_goal_response)

    def _on_nav2_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Nav2 目标被拒绝')
            with self.lock:
                self.nav_active = False
                self.nav_path = []
                self.nav_goal = None
            return
        self.nav2_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._on_nav2_result)

    def _on_nav2_result(self, future) -> None:
        self.nav2_goal_handle = None
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Nav2 已到达目标点')
            self._advance_nav_queue()
        elif status != GoalStatus.STATUS_CANCELED:
            # 中止/失败：结束本次导航（取消由 nav_stop 触发，状态已清理）
            self.get_logger().warning(f'Nav2 导航中止（状态 {status}）')
            with self.lock:
                self.nav_active = False
                self.nav_path = []
                self.nav_goal = None

    def _cancel_nav2_goal(self) -> None:
        """取消正在执行的 Nav2 导航目标（有则取消，无则跳过）。"""
        if self.nav2_goal_handle is not None:
            self.nav2_goal_handle.cancel_goal_async()
            self.nav2_goal_handle = None

    def _on_image(self, message: Image, rear: bool = False) -> None:
        now = time.monotonic()
        image_time = self.rear_image_time if rear else self.image_time
        if now - image_time < self.camera_interval:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            success, encoded = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            )
        except Exception as error:
            self.get_logger().warning(f"Camera conversion failed: {error}")
            return
        if success:
            with self.lock:
                if rear:
                    self.rear_image_jpeg = encoded.tobytes()
                    self.rear_image_time = now
                    self.topic_times["camera_rear"] = now
                else:
                    self.image_jpeg = encoded.tobytes()
                    self.image_time = now
                    self.topic_times["camera"] = now

    # ---------------- teleop safety ----------------
    def publish_teleop(self, linear: float, angular: float) -> None:
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.teleop_publisher.publish(command)
        self.last_teleop_time = time.monotonic()
        heartbeat = Bool()
        heartbeat.data = True
        self.heartbeat_publisher.publish(heartbeat)

    def _teleop_watchdog(self) -> None:
        now = time.monotonic()
        active = bool(
            self.last_teleop_time and now - self.last_teleop_time <= self.teleop_watchdog
        )
        if self.last_teleop_time and not active:
            # Operator channel went silent: command a stop exactly once.
            self.teleop_publisher.publish(Twist())
            self.last_teleop_time = 0.0
        heartbeat = Bool()
        heartbeat.data = active
        self.heartbeat_publisher.publish(heartbeat)

    # ---------------- cruise switch ----------------
    def _poll_cruise_state(self) -> None:
        """2s 轮询避障节点 enable_cruise 现值（外部 ros2 param set 也能反映）。"""
        if not self.cruise_get_client.service_is_ready():
            with self.lock:
                self.cruise_state = None
            return
        request = GetParameters.Request()
        request.names = ["enable_cruise"]
        future = self.cruise_get_client.call_async(request)

        def on_done(fut):
            try:
                response = fut.result()
                if response.values:
                    with self.lock:
                        self.cruise_state = bool(response.values[0].bool_value)
                    self._sync_steering_assist(self.cruise_state)
            except Exception:
                pass

        future.add_done_callback(on_done)

    def _sync_steering_assist(self, cruise_enabled) -> None:
        """巡航状态变化时同步 mux 的 steering_assist 参数（失败则下轮重试）。"""
        if not isinstance(cruise_enabled, bool):
            return
        if cruise_enabled == self._assist_pushed:
            return
        if not self.assist_set_client.service_is_ready():
            return
        parameter = Parameter(
            name="steering_assist",
            value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=cruise_enabled),
        )
        request = SetParameters.Request()
        request.parameters = [parameter]
        future = self.assist_set_client.call_async(request)

        def on_done(fut):
            try:
                response = fut.result()
                if response.results and response.results[0].successful:
                    self._assist_pushed = cruise_enabled
                    self.get_logger().info(
                        f"mux steering_assist <- {cruise_enabled}")
            except Exception:
                pass

        future.add_done_callback(on_done)

    def set_cruise(self, enable: bool):
        """HTTP 线程调用：设置 enable_cruise。返回 (ok, error)。"""
        if not self.cruise_set_client.service_is_ready():
            return False, "avoidance_unavailable"
        parameter = Parameter(
            name="enable_cruise",
            value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=bool(enable)),
        )
        request = SetParameters.Request()
        request.parameters = [parameter]
        future = self.cruise_set_client.call_async(request)
        done = threading.Event()
        outcome = {}

        def on_done(fut):
            try:
                response = fut.result()
                outcome["ok"] = bool(
                    response.results and response.results[0].successful)
                if response.results:
                    outcome["reason"] = response.results[0].reason
            except Exception as error:
                outcome["ok"] = False
                outcome["reason"] = str(error)
            done.set()

        future.add_done_callback(on_done)
        if not done.wait(2.0):
            return False, "set_timeout"
        if outcome.get("ok"):
            with self.lock:
                self.cruise_state = bool(enable)
            self._sync_steering_assist(self.cruise_state)
            return True, ""
        return False, outcome.get("reason") or "set_rejected"

    # ---------------- one-click mapping ----------------
    def set_mapping(self, enable: bool, auto_cruise: bool = True):
        """HTTP 线程调用：启动/停止建图流水线。返回 (ok, message)。

        启动 = slam_toolbox + 可选自动巡航（巡航 wander 让小车自主在区域内
        走动，SLAM 同步建图）；base_footprint→laser_frame 静态 TF 由
        real_bringup 常驻发布，建图不再单独启动。
        停止 = 退巡航 → map_saver_cli 保存 ~/maps/map_<时间戳> → 终止子进程。
        """
        if enable == self.mapping_active:
            return True, "already_active" if enable else "already_inactive"
        if enable:
            procs = []
            try:
                # start_new_session：子进程独立进程组，停止时按组发信号——
                # ros2 CLI 包装脚本会吞掉 SIGINT，直接 signal 包装进程杀不死节点
                procs.append(subprocess.Popen([
                    "ros2", "run", "slam_toolbox", "async_slam_toolbox_node",
                    "--ros-args",
                    "-p", "base_frame:=base_footprint",
                    "-p", "odom_frame:=odom",
                    "-p", "scan_topic:=/scan"],
                    start_new_session=True))
            except FileNotFoundError:
                for proc in procs:
                    proc.terminate()
                return False, "slam_toolbox_not_installed"
            self.mapping_procs = procs
            self.mapping_active = True
            self.mapping_started = time.monotonic()
            self.get_logger().info("建图流水线已启动（slam_toolbox）")
            if auto_cruise:
                ok, error = self.set_cruise(True)
                if not ok:
                    self.get_logger().warning(
                        f"建图已启动，但自动巡航开启失败：{error}（可手动遥控）")
            return True, "started"
        # 停止：先退巡航 → 保存地图（需 slam_toolbox 还活着）→ 终止子进程
        if auto_cruise:
            self.set_cruise(False)
        saved = self._save_map()
        for proc in self.mapping_procs:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
        for proc in self.mapping_procs:
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self.mapping_procs = []
        self.mapping_active = False
        self.mapping_started = 0.0
        return True, saved or "stopped"

    def _save_map(self):
        """用 map_saver_cli 把当前 /map 存到 ~/maps/，返回保存路径前缀或 None。"""
        with self.lock:
            has_map = self.map_png is not None
        if not has_map:
            return None
        maps_dir = os.path.expanduser("~/maps")
        os.makedirs(maps_dir, exist_ok=True)
        path = os.path.join(
            maps_dir, "map_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
        try:
            subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli",
                 "-f", path, "-t", "/map"],
                timeout=20.0, capture_output=True)
        except Exception as error:
            self.get_logger().warning(f"地图保存失败：{error}")
            return None
        self.get_logger().info(f"地图已保存：{path}.pgm/.yaml")
        return path

    # ---------------- HTTP ----------------
    def _handler_class(self):
        gateway = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "CarWebGateway/1.0"

            def log_message(self, format_string, *args):
                gateway.get_logger().debug(format_string % args)

            def _headers(
                self, status: int, content_type: str, length: Optional[int] = None
            ):
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header("Referrer-Policy", "no-referrer")
                if length is not None:
                    self.send_header("Content-Length", str(length))
                self.end_headers()

            def _json(self, status: int, body: dict):
                encoded = json.dumps(body, ensure_ascii=False).encode("utf-8")
                try:
                    self._headers(
                        status, "application/json; charset=utf-8", len(encoded)
                    )
                    self.wfile.write(encoded)
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    # Browsers routinely cancel superseded status/image
                    # requests; that is not a gateway fault.
                    pass

            def do_GET(self):
                path = urlparse(self.path).path
                if path in ("/", "/index.html"):
                    gateway._serve_index(self)
                    return
                if path == "/api/health":
                    self._json(HTTPStatus.OK, gateway.health_snapshot())
                    return
                if path == "/api/status":
                    self._json(HTTPStatus.OK, gateway.status_snapshot())
                    return
                if path == "/api/scan.json":
                    self._json(HTTPStatus.OK, gateway.scan_snapshot())
                    return
                if path == "/api/map.png":
                    gateway._serve_map(self)
                    return
                if path == "/api/maps":
                    self._json(HTTPStatus.OK, gateway.maps_snapshot())
                    return
                if path.startswith("/api/maps/") and path.endswith(".png"):
                    gateway._serve_saved_map(self, path[10:-4])
                    return
                if path == "/api/camera.jpg":
                    gateway._serve_camera(self, rear=False)
                    return
                if path == "/api/camera_rear.jpg":
                    gateway._serve_camera(self, rear=True)
                    return
                if path == "/api/photo/download":
                    gateway._serve_photo_download(self)
                    return
                if path == "/api/nav/status":
                    self._json(HTTPStatus.OK, gateway.nav_snapshot())
                    return
                if path == "/api/auto_cruise/status":
                    self._json(HTTPStatus.OK, gateway.auto_cruise_snapshot())
                    return
                self._json(HTTPStatus.NOT_FOUND, {"error": "not_found"})

            def do_POST(self):
                path = urlparse(self.path).path
                if path not in (
                    "/api/ugv/teleop", "/api/ugv/cruise", "/api/mapping",
                    "/api/photo", "/api/nav/set_goal", "/api/nav/stop",
                    "/api/nav/load_map", "/api/auto_cruise/start",
                    "/api/auto_cruise/stop",
                ):
                    self._json(HTTPStatus.NOT_FOUND, {"error": "unknown_command"})
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0") or 0)
                except ValueError:
                    length = -1
                if length < 0 or length > 4096:
                    self._json(
                        HTTPStatus.REQUEST_ENTITY_TOO_LARGE,
                        {"error": "request_too_large"},
                    )
                    return
                try:
                    payload = (
                        json.loads(self.rfile.read(length).decode("utf-8"))
                        if length
                        else {}
                    )
                except (UnicodeDecodeError, json.JSONDecodeError):
                    self._json(HTTPStatus.BAD_REQUEST, {"error": "invalid_json"})
                    return
                if path == "/api/ugv/cruise":
                    self._handle_cruise(payload)
                    return
                if path == "/api/mapping":
                    self._handle_mapping(payload)
                    return
                if path == "/api/photo":
                    self._handle_photo()
                    return
                if path == "/api/nav/set_goal":
                    self._handle_nav_set_goal(payload)
                    return
                if path == "/api/nav/stop":
                    self._handle_nav_stop()
                    return
                if path == "/api/nav/load_map":
                    self._handle_nav_load_map(payload)
                    return
                if path == "/api/auto_cruise/start":
                    self._handle_auto_cruise_start()
                    return
                if path == "/api/auto_cruise/stop":
                    self._handle_auto_cruise_stop()
                    return
                try:
                    linear, angular = clamped_teleop(
                        payload, gateway.max_linear, gateway.max_angular
                    )
                except (TypeError, ValueError) as error:
                    self._json(HTTPStatus.BAD_REQUEST, {"error": str(error)})
                    return
                gateway.publish_teleop(linear, angular)
                self._json(
                    HTTPStatus.ACCEPTED,
                    {
                        "accepted": True,
                        "status": "teleop_published",
                        "linear": linear,
                        "angular": angular,
                    },
                )

            def _handle_cruise(self, payload: dict):
                enable = payload.get("enable") if isinstance(payload, dict) else None
                if not isinstance(enable, bool):
                    self._json(HTTPStatus.BAD_REQUEST, {"error": "enable_bool_required"})
                    return
                ok, error = gateway.set_cruise(enable)
                if not ok:
                    status = (
                        HTTPStatus.SERVICE_UNAVAILABLE
                        if error == "avoidance_unavailable"
                        else HTTPStatus.BAD_GATEWAY
                    )
                    self._json(status, {"error": error})
                    return
                self._json(
                    HTTPStatus.ACCEPTED,
                    {"accepted": True, "cruise_enabled": enable},
                )

            def _handle_mapping(self, payload: dict):
                enable = payload.get("enable") if isinstance(payload, dict) else None
                if not isinstance(enable, bool):
                    self._json(
                        HTTPStatus.BAD_REQUEST, {"error": "enable_bool_required"})
                    return
                auto_cruise = bool(payload.get("auto_cruise", True))
                ok, message = gateway.set_mapping(enable, auto_cruise)
                if not ok:
                    self._json(HTTPStatus.BAD_GATEWAY, {"error": message})
                    return
                self._json(
                    HTTPStatus.ACCEPTED,
                    {"accepted": True, "mapping": enable, "message": message},
                )

            def _handle_photo(self):
                ok, result = gateway.save_photo_frame()
                if not ok:
                    status = (
                        HTTPStatus.SERVICE_UNAVAILABLE
                        if result == "camera_not_ready"
                        else HTTPStatus.INTERNAL_SERVER_ERROR
                    )
                    self._json(status, {"error": result})
                    return
                self._json(
                    HTTPStatus.ACCEPTED, {"accepted": True, "path": result})

            def _handle_nav_set_goal(self, payload: dict):
                if not isinstance(payload, dict):
                    self._json(HTTPStatus.BAD_REQUEST, {"error": "json_object_required"})
                    return
                x = payload.get("x")
                y = payload.get("y")
                if x is None or y is None:
                    self._json(HTTPStatus.BAD_REQUEST, {"error": "x_y_required"})
                    return
                try:
                    x = float(x)
                    y = float(y)
                except (TypeError, ValueError):
                    self._json(HTTPStatus.BAD_REQUEST, {"error": "invalid_coordinate"})
                    return
                # 可选初始点
                start_x = payload.get("start_x")
                start_y = payload.get("start_y")
                if start_x is not None and start_y is not None:
                    try:
                        start_x = float(start_x)
                        start_y = float(start_y)
                    except (TypeError, ValueError):
                        self._json(HTTPStatus.BAD_REQUEST, {"error": "invalid_start_coordinate"})
                        return
                else:
                    start_x = start_y = None
                ok, result = gateway.nav_set_goal(x, y, start_x, start_y)
                if not ok:
                    self._json(HTTPStatus.BAD_GATEWAY, {"error": result})
                    return
                self._json(
                    HTTPStatus.ACCEPTED,
                    {"accepted": True, "path": result, "goal": [x, y]},
                )

            def _handle_nav_stop(self):
                gateway.nav_stop()
                self._json(HTTPStatus.ACCEPTED, {"accepted": True})

            def _handle_nav_load_map(self, payload: dict):
                if not isinstance(payload, dict):
                    self._json(HTTPStatus.BAD_REQUEST, {"error": "json_object_required"})
                    return
                map_name = payload.get("name")
                if not map_name:
                    self._json(HTTPStatus.BAD_REQUEST, {"error": "name_required"})
                    return
                ok = gateway._load_map(str(map_name))
                if not ok:
                    self._json(HTTPStatus.BAD_GATEWAY, {"error": "load_failed"})
                    return
                self._json(HTTPStatus.ACCEPTED, {"accepted": True, "map": map_name})

            def _handle_auto_cruise_start(self):
                ok, message = gateway.auto_cruise_start()
                if not ok:
                    self._json(HTTPStatus.BAD_GATEWAY, {"error": message})
                    return
                self._json(HTTPStatus.ACCEPTED, {"accepted": True, "message": message})

            def _handle_auto_cruise_stop(self):
                gateway.auto_cruise_stop()
                self._json(HTTPStatus.ACCEPTED, {"accepted": True})

        return Handler

    def _serve_index(self, handler) -> None:
        try:
            with open(_index_path(), "rb") as stream:
                body = stream.read()
        except OSError:
            handler._json(
                HTTPStatus.NOT_FOUND, {"error": "index_page_not_installed"}
            )
            return
        try:
            handler._headers(HTTPStatus.OK, "text/html; charset=utf-8", len(body))
            handler.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def _serve_map(self, handler) -> None:
        with self.lock:
            image = self.map_png
        if image is None:
            handler._json(
                HTTPStatus.SERVICE_UNAVAILABLE, {"error": "map_not_ready"}
            )
            return
        try:
            handler._headers(HTTPStatus.OK, "image/png", len(image))
            handler.wfile.write(image)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    _MAPS_DIR = os.path.expanduser("~/maps")
    _PHOTOS_DIR = os.path.expanduser("~/photos")

    def save_photo_frame(self) -> Tuple[bool, str]:
        """保存当前前摄帧到 ~/photos/；返回 (是否成功, 路径或错误码)。"""
        with self.lock:
            image = self.image_jpeg
        if image is None:
            return False, "camera_not_ready"
        try:
            return True, save_photo(image, self._PHOTOS_DIR)
        except OSError:
            return False, "save_failed"

    def maps_snapshot(self) -> dict:
        """~/maps 下已保存的地图列表（新的在前）。"""
        entries = []
        try:
            for fname in sorted(os.listdir(self._MAPS_DIR), reverse=True):
                if fname.endswith(".pgm"):
                    entries.append(fname[:-4])
        except OSError:
            pass
        return {"maps": entries}

    def _serve_saved_map(self, handler, name: str) -> None:
        """把 ~/maps/<name>.pgm 转 PNG 输出（只接受安全文件名，防目录穿越）。"""
        if not name or not all(c.isalnum() or c in "_-" for c in name):
            handler._json(HTTPStatus.BAD_REQUEST, {"error": "invalid_name"})
            return
        path = os.path.join(self._MAPS_DIR, name + ".pgm")
        if cv2 is None or not os.path.isfile(path):
            handler._json(HTTPStatus.NOT_FOUND, {"error": "map_not_found"})
            return
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            handler._json(HTTPStatus.NOT_FOUND, {"error": "map_unreadable"})
            return
        # map_saver 的 pgm：0=占据 254=空闲 205=未知；翻转为上北显示
        img = cv2.flip(img, 0)
        success, encoded = cv2.imencode(".png", img)
        if not success:
            handler._json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": "encode_failed"})
            return
        try:
            handler._headers(HTTPStatus.OK, "image/png", len(encoded))
            handler.wfile.write(encoded.tobytes())
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def _serve_camera(self, handler, rear: bool = False) -> None:
        if not _CAMERA_AVAILABLE:
            handler._json(
                HTTPStatus.SERVICE_UNAVAILABLE, {"error": "camera_backend_unavailable"}
            )
            return
        with self.lock:
            image = self.rear_image_jpeg if rear else self.image_jpeg
        if image is None:
            handler._json(
                HTTPStatus.SERVICE_UNAVAILABLE, {"error": "camera_not_ready"}
            )
            return
        try:
            handler._headers(HTTPStatus.OK, "image/jpeg", len(image))
            handler.wfile.write(image)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def _serve_photo_download(self, handler) -> None:
        """把最新前摄帧作为可下载文件返回给浏览器客户端。"""
        if not _CAMERA_AVAILABLE:
            handler._json(
                HTTPStatus.SERVICE_UNAVAILABLE,
                {"error": "camera_backend_unavailable"},
            )
            return
        with self.lock:
            image = self.image_jpeg
        if image is None:
            handler._json(
                HTTPStatus.SERVICE_UNAVAILABLE, {"error": "camera_not_ready"}
            )
            return
        stamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"photo_{stamp}.jpg"
        try:
            handler.send_response(HTTPStatus.OK)
            handler.send_header("Content-Type", "image/jpeg")
            handler.send_header("Content-Length", str(len(image)))
            handler.send_header(
                "Content-Disposition",
                f'attachment; filename="{filename}"',
            )
            handler.end_headers()
            handler.wfile.write(image)
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    # ---------------- 自主导航 ----------------
    def _load_map(self, map_name: str):
        """加载 ~/maps/<name>.pgm 和 .yaml，返回是否成功。"""
        if not map_name or not all(c.isalnum() or c in "_-" for c in map_name):
            return False
        pgm_path = os.path.join(self._MAPS_DIR, map_name + ".pgm")
        yaml_path = os.path.join(self._MAPS_DIR, map_name + ".yaml")
        if not os.path.isfile(pgm_path) or not os.path.isfile(yaml_path):
            return False
        if cv2 is None:
            return False
        import numpy as np
        import yaml
        try:
            with open(yaml_path, "r") as f:
                meta = yaml.safe_load(f)
            img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                return False
            # pgm：0=占据 254=空闲 205=未知；翻转回地图坐标系
            img = cv2.flip(img, 0)
            # 转换为占用概率：0=空闲 100=占据 -1=未知
            data = np.full(img.shape, -1, dtype=np.int8)
            data[img == 254] = 0    # 空闲
            data[img == 0] = 100    # 占据
            data[img == 205] = -1   # 未知
            self.nav_map_data = data
            self.nav_map_resolution = float(meta.get("resolution", 0.05))
            origin = meta.get("origin", [0.0, 0.0, 0.0])
            self.nav_map_origin = (float(origin[0]), float(origin[1]))
            self.get_logger().info(
                f'地图 {map_name} 已加载：{data.shape[1]}x{data.shape[0]}，'
                f'分辨率 {self.nav_map_resolution:.3f}m')
            return True
        except Exception as e:
            self.get_logger().error(f'加载地图失败：{e}')
            return False

    def _world_to_grid(self, x: float, y: float):
        """odom 系坐标 → 地图栅格坐标。"""
        if self.nav_map_data is None or self.nav_map_origin is None:
            return None
        gx = int((x - self.nav_map_origin[0]) / self.nav_map_resolution)
        gy = int((y - self.nav_map_origin[1]) / self.nav_map_resolution)
        if 0 <= gx < self.nav_map_data.shape[1] and \
                0 <= gy < self.nav_map_data.shape[0]:
            return (gx, gy)
        return None

    def _grid_to_world(self, gx: int, gy: int):
        """地图栅格坐标 → odom 系坐标。"""
        if self.nav_map_data is None or self.nav_map_origin is None:
            return None
        x = self.nav_map_origin[0] + (gx + 0.5) * self.nav_map_resolution
        y = self.nav_map_origin[1] + (gy + 0.5) * self.nav_map_resolution
        return (x, y)

    def _is_traversable(self, gx: int, gy: int) -> bool:
        """栅格是否可通行：空闲且不在地图边界外。"""
        if self.nav_map_data is None:
            return False
        if gx < 0 or gy < 0 or gx >= self.nav_map_data.shape[1] or \
                gy >= self.nav_map_data.shape[0]:
            return False
        return self.nav_map_data[gy, gx] == 0

    def _plan_path(self, start, goal):
        """BFS 在地图上规划从 start 到 goal 的路径（栅格坐标）。

        返回栅格坐标路径列表，无法规划返回 None。
        """
        if self.nav_map_data is None:
            return None
        start_g = self._world_to_grid(*start)
        goal_g = self._world_to_grid(*goal)
        if start_g is None or goal_g is None:
            return None
        if not self._is_traversable(*start_g) or \
                not self._is_traversable(*goal_g):
            return None
        visited = {start_g}
        queue = deque([(start_g, [start_g])])
        while queue:
            (gx, gy), path = queue.popleft()
            if (gx, gy) == goal_g:
                return path
            for dx, dy in ((0, 1), (1, 0), (0, -1), (-1, 0),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nx, ny = gx + dx, gy + dy
                if (nx, ny) in visited or not self._is_traversable(nx, ny):
                    continue
                visited.add((nx, ny))
                queue.append(((nx, ny), path + [(nx, ny)]))
        return None

    def _smooth_path(self, path):
        """简化路径：移除共线点，只保留拐点。"""
        if len(path) <= 2:
            return path
        simplified = [path[0]]
        for i in range(1, len(path) - 1):
            prev = path[i - 1]
            curr = path[i]
            next_pt = path[i + 1]
            # 检查是否共线
            dx1, dy1 = curr[0] - prev[0], curr[1] - prev[1]
            dx2, dy2 = next_pt[0] - curr[0], next_pt[1] - curr[1]
            if dx1 * dy2 != dy1 * dx2:  # 不共线
                simplified.append(curr)
        simplified.append(path[-1])
        return simplified

    def nav_set_goal(self, goal_x: float, goal_y: float, start_x: float = None, start_y: float = None):
        """设置导航目标点，返回 (是否成功, 路径或错误码)。

        若提供 start_x/start_y，则从该点规划路径；否则从当前位置规划。
        nav2 模式下全局规划由 NavFn 承担，本地 BFS 路径规划跳过，
        目标点直接经 NavigateToPose action 下发（返回空路径）。
        """
        if self.nav_backend == "nav2":
            if not self.nav2_client.server_is_ready():
                return False, "nav2_not_ready"
            with self.lock:
                # 单目标直发 Nav2，本地路径点队列不启用
                self.nav_path = []
                self.nav_goal = (goal_x, goal_y)
                self.nav_active = True
            self._send_nav2_goal((goal_x, goal_y))
            self.get_logger().info(
                f'Nav2 目标已下发：({goal_x:.2f}, {goal_y:.2f})')
            return True, []
        if start_x is None or start_y is None:
            if self.pose is None:
                return False, "no_pose"
            start = self.pose[:2]
        else:
            start = (start_x, start_y)
        if self.nav_map_data is None:
            return False, "no_map"
        # 规划路径
        path_grid = self._plan_path(start, (goal_x, goal_y))
        if path_grid is None:
            return False, "no_path"
        # 转换为 odom 系坐标
        path_world = [self._grid_to_world(gx, gy) for gx, gy in path_grid]
        path_world = [p for p in path_world if p is not None]
        if not path_world:
            return False, "path_conversion_failed"
        # 简化路径
        path_world = self._smooth_path(path_world)
        with self.lock:
            self.nav_path = path_world
            self.nav_goal = (goal_x, goal_y)
            self.nav_active = True
        self.get_logger().info(
            f'导航路径已规划：{len(path_world)} 点，终点 ({goal_x:.2f}, '
            f'{goal_y:.2f})')
        # 发布第一个目标点给避障节点
        if path_world:
            self._publish_nav_goal(path_world[0])
        return True, path_world

    def _publish_nav_goal(self, goal):
        """发布导航目标点：avoidance 模式发 /goal_pose 话题（odom 系），
        nav2 模式发 NavigateToPose action（map 系）。"""
        if self.nav_backend == "nav2":
            self._send_nav2_goal(goal)
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.orientation.w = 1.0
        self.nav_goal_publisher.publish(msg)

    def nav_stop(self):
        """停止自主导航：清空本地路径队列，取消避障节点/Nav2 的当前目标。"""
        with self.lock:
            self.nav_active = False
            self.nav_path = []
            self.nav_goal = None
        # 通知避障节点清除正在跟踪的目标点（否则心跳过期后它会继续
        # 朝旧目标行驶，表现为"停不下来"）
        cancel = Bool()
        cancel.data = True
        self.nav_cancel_publisher.publish(cancel)
        # Nav2 模式：取消正在执行的导航 action
        self._cancel_nav2_goal()
        self.get_logger().info('自主导航已停止')

    def nav_snapshot(self):
        """返回当前导航状态。"""
        with self.lock:
            return {
                "active": self.nav_active,
                "path": self.nav_path,
                "goal": self.nav_goal,
                "has_map": self.nav_map_data is not None,
            }

    # ---------------- 自动巡航 ----------------
    def auto_cruise_start(self):
        """开始自动巡航：小车从初始点出发，绕地图可行走区域一圈后回到初始点。"""
        if self.nav_map_data is None:
            return False, "no_map"
        if self.pose is None:
            return False, "no_pose"
        # 预设初始点：地图中心（可通行区域中心）
        if not self._find_initial_point():
            return False, "no_initial_point"
        # 开始自动巡航：先回到初始点，然后开始绕圈
        self.auto_cruise_active = True
        self.auto_cruise_started = time.monotonic()
        self.auto_cruise_returning = False
        # 规划绕地图一圈的路径
        if not self._plan_cruise_path():
            self.auto_cruise_active = False
            return False, "no_cruise_path"
        self.get_logger().info('自动巡航已开始')
        return True, "started"

    def auto_cruise_stop(self):
        """停止自动巡航，并发送停止运动指令。"""
        self.auto_cruise_active = False
        self.nav_stop()
        # 发送停止运动指令，确保小车立即停止
        self.publish_teleop(0.0, 0.0)
        self.get_logger().info('自动巡航已停止，小车已停止运动')

    def _plan_cruise_path(self):
        """规划绕地图可行走区域一圈的路径。"""
        if self.nav_map_data is None:
            return False
        import numpy as np
        # 找到所有可通行栅格
        traversable = np.where(self.nav_map_data == 0)
        if len(traversable[0]) == 0:
            return False
        # 找到可通行区域的边界
        min_gx, max_gx = np.min(traversable[1]), np.max(traversable[1])
        min_gy, max_gy = np.min(traversable[0]), np.max(traversable[0])
        # 规划一条绕边界的路径：从初始点出发，沿边界顺时针走一圈
        path = []
        # 从初始点开始
        start_g = self._world_to_grid(*self.auto_cruise_initial)
        if start_g is None:
            return False
        # 沿边界顺时针走一圈
        # 上边：从左到右
        for gx in range(min_gx, max_gx + 1):
            if self._is_traversable(gx, min_gy):
                path.append((gx, min_gy))
        # 右边：从上到下
        for gy in range(min_gy + 1, max_gy + 1):
            if self._is_traversable(max_gx, gy):
                path.append((max_gx, gy))
        # 下边：从右到左
        for gx in range(max_gx - 1, min_gx - 1, -1):
            if self._is_traversable(gx, max_gy):
                path.append((gx, max_gy))
        # 左边：从下到上
        for gy in range(max_gy - 1, min_gy, -1):
            if self._is_traversable(min_gx, gy):
                path.append((min_gx, gy))
        if not path:
            return False
        # 转换为 odom 系坐标
        path_world = [self._grid_to_world(gx, gy) for gx, gy in path]
        path_world = [p for p in path_world if p is not None]
        if not path_world:
            return False
        # 简化路径
        path_world = self._smooth_path(path_world)
        with self.lock:
            self.nav_path = path_world
            self.nav_goal = self.auto_cruise_initial
            self.nav_active = True
        self.get_logger().info(
            f'自动巡航路径已规划：{len(path_world)} 点')
        # 发布第一个目标点给避障节点
        if path_world:
            self._publish_nav_goal(path_world[0])
        return True

    def _find_initial_point(self):
        """在地图上找到可行走区域的中心作为初始点。"""
        if self.nav_map_data is None:
            return False
        import numpy as np
        # 找到所有可通行栅格
        traversable = np.where(self.nav_map_data == 0)
        if len(traversable[0]) == 0:
            return False
        # 计算可通行区域中心
        center_gx = int(np.mean(traversable[1]))
        center_gy = int(np.mean(traversable[0]))
        # 转换为 odom 系坐标
        initial = self._grid_to_world(center_gx, center_gy)
        if initial is None:
            return False
        self.auto_cruise_initial = initial
        self.get_logger().info(
            f'初始点已设置：({initial[0]:.2f}, {initial[1]:.2f})')
        return True

    def auto_cruise_snapshot(self):
        """返回自动巡航状态。"""
        # 如果初始点未设置，尝试查找
        if self.auto_cruise_initial is None and self.nav_map_data is not None:
            self._find_initial_point()
        return {
            "active": self.auto_cruise_active,
            "initial": self.auto_cruise_initial,
            "map_name": self.nav_map_name,
            "map_info": {
                "width": self.nav_map_data.shape[1] if self.nav_map_data is not None else 0,
                "height": self.nav_map_data.shape[0] if self.nav_map_data is not None else 0,
                "resolution": self.nav_map_resolution,
                "origin": self.nav_map_origin,
            } if self.nav_map_data is not None else None,
        }

    # ---------------- snapshots ----------------
    def scan_snapshot(self) -> dict:
        now = time.monotonic()
        with self.lock:
            age = None if not self.scan_time else round(now - self.scan_time, 2)
            points = self.scan_points
            range_max = self.scan_range_max
        return {"age_s": age, "range_max": range_max, "points": points}

    def health_snapshot(self) -> dict:
        return {
            "ok": True,
            "schema_version": "1.0",
            "service": "car-web-gateway",
            "uptime_s": round(time.monotonic() - self.started_at, 1),
            "camera_backend": _CAMERA_AVAILABLE,
        }

    def _lookup_map_pose(self):
        """查询 map→base_footprint 变换（AMCL 定位），返回 [x, y, yaw]；
        无定位输出（avoidance 模式或未收敛）时为 None。"""
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        translation = transform.transform.translation
        yaw = _yaw_from_quaternion(transform.transform.rotation)
        return [round(translation.x, 3), round(translation.y, 3), round(yaw, 3)]

    def status_snapshot(self) -> dict:
        now = time.monotonic()
        map_pose = self._lookup_map_pose()
        with self.lock:
            mux = dict(self.latest["ugv_control_mux"])
            gateway_status = dict(self.latest["ugv_command_gateway"])
            pose = list(self.pose) if self.pose is not None else None
            speed = self.speed_mps
            cruise_state = self.cruise_state
            topic_ages = {
                key: round(now - timestamp, 2)
                for key, timestamp in self.topic_times.items()
            }
            image_age = None if not self.image_time else round(now - self.image_time, 2)
            rear_image_age = (
                None if not self.rear_image_time else round(now - self.rear_image_time, 2)
            )
            fix = dict(self.fix) if self.fix is not None else None
            fix_age = None if not self.fix_time else round(now - self.fix_time, 2)
            map_info = dict(self.map_info) if self.map_info is not None else None
            map_age = None if not self.map_time else round(now - self.map_time, 2)
        return {
            "gateway": self.health_snapshot(),
            "server_time_ms": int(time.time() * 1000),
            "ugv_control_mux": mux,
            "ugv_command_gateway": gateway_status,
            "avoidance": {
                "available": cruise_state is not None,
                "cruise_enabled": cruise_state,
            },
            "odom": {"pose": pose, "speed_mps": speed},
            "localization": {
                "backend": self.nav_backend,
                "map_pose": map_pose,
            },
            "battery_voltage": None,  # reserved for the real motor driver
            "gps": {"fix": fix, "age_s": fix_age},
            "map": {"info": map_info, "age_s": map_age},
            "mapping": {
                "active": self.mapping_active,
                "since_s": (
                    None if not self.mapping_started
                    else round(now - self.mapping_started, 1)
                ),
            },
            "camera": {
                "ready": bool(
                    image_age is not None and image_age <= self.camera_stale_after
                ),
                "age_s": image_age,
            },
            "camera_rear": {
                "ready": bool(
                    rear_image_age is not None
                    and rear_image_age <= self.camera_stale_after
                ),
                "age_s": rear_image_age,
            },
            "topic_ages_s": topic_ages,
        }

    def destroy_node(self):
        for proc in getattr(self, "mapping_procs", []):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass
        if rclpy.ok():
            try:
                self.teleop_publisher.publish(Twist())
                heartbeat = Bool()
                heartbeat.data = False
                self.heartbeat_publisher.publish(heartbeat)
            except Exception:
                # The ROS context may already be closing during launch
                # shutdown; runtime watchdogs remain the authoritative stop.
                pass
        self.http_server.shutdown()
        self.http_server.server_close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
