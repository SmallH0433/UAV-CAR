"""LAN-friendly, allow-listed HTTP/SSE gateway for the air-ground dashboard."""

from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hmac
import json
import math
import os
import queue
import subprocess
import threading
import time
from typing import Any, Dict, Optional
from urllib.parse import urlparse
import uuid

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Vector3
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from .runtime_timing import create_steady_timer
from .ros_compat import RCLError

from .audit_journal import AuditJournal
from .security_policy import (
    control_token_is_strong,
    production_motion_plan_allows,
    trusted_client_address,
    trusted_proxy_identity,
)


CAMERA_TOPICS = {
    "downward": "/vision/image_raw",
    "landing": "/apriltag/debug_image",
    "stereo_left": "/uav/stereo/left/image_raw",
    "stereo_right": "/uav/stereo/right/image_raw",
    "ugv": "/ugv/camera/image_raw",
}


def requested_camera_keys(
    dashboard_clients: int,
    interest_until: Dict[str, float],
    now_s: float,
) -> set[str]:
    """Return streams that should consume image and rendering resources."""

    if dashboard_clients > 0:
        return set(CAMERA_TOPICS)
    return {
        key
        for key, deadline in interest_until.items()
        if key in CAMERA_TOPICS and float(deadline) > float(now_s)
    }


def camera_frame_is_fresh(
    timestamp: Optional[float], now_s: float, stale_after_s: float
) -> bool:
    """Return whether a JPEG cache entry is recent enough to serve as live video."""

    return (
        timestamp is not None
        and float(now_s) >= float(timestamp)
        and float(now_s) - float(timestamp) <= float(stale_after_s)
    )


def camera_qos_profile(reliability: str) -> QoSProfile:
    """Use depth one so live video never queues stale frames."""

    normalized = str(reliability).strip().lower()
    if normalized not in {"reliable", "best_effort"}:
        raise ValueError(
            "camera_qos_reliability must be 'reliable' or 'best_effort'"
        )
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=(
            ReliabilityPolicy.RELIABLE
            if normalized == "reliable"
            else ReliabilityPolicy.BEST_EFFORT
        ),
        durability=DurabilityPolicy.VOLATILE,
    )


@dataclass
class GatewayCommand:
    name: str
    payload: Dict[str, Any]
    request_id: str = ""
    operator_id: str = ""
    completed: threading.Event = field(default_factory=threading.Event)
    result: Dict[str, Any] = field(default_factory=dict)


def _json_message(message: String) -> dict:
    try:
        value = json.loads(message.data)
        return value if isinstance(value, dict) else {}
    except json.JSONDecodeError:
        return {}


def _yaw_from_quaternion(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


class DashboardGateway(Node):
    """Aggregate ROS state and execute a small, auditable command allow-list."""

    def __init__(self) -> None:
        super().__init__("web_gateway")
        self.declare_parameter("bind_address", "127.0.0.1")
        self.declare_parameter("port", 8765)
        self.declare_parameter("command_enabled", False)
        self.declare_parameter("auth_token", "")
        self.declare_parameter("cors_origin", "http://127.0.0.1:3000")
        self.declare_parameter("trust_proxy_headers", False)
        self.declare_parameter("simulation_control_enabled", False)
        self.declare_parameter("simulation_world", "air_ground_cooperative")
        self.declare_parameter("world_bounds_json", "[-15.0, 15.0, -11.0, 11.0]")
        self.declare_parameter("no_fly_zones_json", "[]")
        self.declare_parameter("height_limit_zones_json", "[]")
        self.declare_parameter("manual_uav_min_altitude_m", 0.8)
        self.declare_parameter("manual_uav_max_altitude_m", 8.0)
        self.declare_parameter("camera_max_rate_hz", 5.0)
        self.declare_parameter("camera_stream_idle_timeout_s", 3.0)
        self.declare_parameter("camera_stale_after_s", 3.0)
        self.declare_parameter("camera_encoder_threads", 2)
        self.declare_parameter("executor_threads", 4)
        self.declare_parameter("camera_qos_reliability", "best_effort")
        self.declare_parameter("jpeg_quality", 72)
        self.declare_parameter("production_mode", False)
        self.declare_parameter("auth_token_env", "AIR_GROUND_CONTROL_TOKEN")
        self.declare_parameter("require_request_id", False)
        self.declare_parameter("require_operator_id", False)
        self.declare_parameter("max_request_bytes", 16384)
        self.declare_parameter("command_rate_limit_per_minute", 120)
        self.declare_parameter("system_health_timeout_s", 1.5)
        self.declare_parameter(
            "audit_log_path", "~/.local/state/air-ground/gateway-audit.jsonl"
        )
        self.declare_parameter("audit_required", False)
        self.declare_parameter("audit_log_max_bytes", 20 * 1024 * 1024)

        self.bind_address = str(self.get_parameter("bind_address").value)
        self.port = int(self.get_parameter("port").value)
        self.command_enabled = bool(self.get_parameter("command_enabled").value)
        configured_token = str(self.get_parameter("auth_token").value)
        token_env = str(self.get_parameter("auth_token_env").value)
        self.auth_token = os.environ.get(token_env, "") or configured_token
        self.cors_origin = str(self.get_parameter("cors_origin").value)
        self.trust_proxy_headers = bool(
            self.get_parameter("trust_proxy_headers").value
        )
        self.simulation_control_enabled = bool(
            self.get_parameter("simulation_control_enabled").value
        )
        self.simulation_world = str(self.get_parameter("simulation_world").value)
        try:
            bounds = json.loads(str(self.get_parameter("world_bounds_json").value))
            if (
                not isinstance(bounds, list)
                or len(bounds) != 4
                or not all(math.isfinite(float(value)) for value in bounds)
            ):
                raise ValueError
            self.world_bounds = tuple(float(value) for value in bounds)
            if not (
                self.world_bounds[0] < self.world_bounds[1]
                and self.world_bounds[2] < self.world_bounds[3]
            ):
                raise ValueError
            self.no_fly_zones = json.loads(
                str(self.get_parameter("no_fly_zones_json").value)
            )
            self.height_limit_zones = json.loads(
                str(self.get_parameter("height_limit_zones_json").value)
            )
            if not isinstance(self.no_fly_zones, list) or not isinstance(
                self.height_limit_zones, list
            ):
                raise ValueError
        except (json.JSONDecodeError, TypeError, ValueError) as error:
            raise ValueError("invalid web gateway world/airspace metadata") from error
        self.manual_uav_min_altitude = float(
            self.get_parameter("manual_uav_min_altitude_m").value
        )
        self.manual_uav_max_altitude = float(
            self.get_parameter("manual_uav_max_altitude_m").value
        )
        if not (
            math.isfinite(self.manual_uav_min_altitude)
            and math.isfinite(self.manual_uav_max_altitude)
            and 0.0 <= self.manual_uav_min_altitude < self.manual_uav_max_altitude
        ):
            raise ValueError("invalid manual UAV altitude envelope")
        camera_rate = max(float(self.get_parameter("camera_max_rate_hz").value), 0.2)
        self.camera_interval = 1.0 / camera_rate
        self.camera_stream_idle_timeout = max(
            1.0,
            float(self.get_parameter("camera_stream_idle_timeout_s").value),
        )
        self.camera_stale_after = max(
            self.camera_interval * 2.0,
            float(self.get_parameter("camera_stale_after_s").value),
        )
        self.camera_encoder_threads = max(
            1, min(4, int(self.get_parameter("camera_encoder_threads").value))
        )
        self.executor_threads = max(
            2, min(8, int(self.get_parameter("executor_threads").value))
        )
        self.camera_qos = camera_qos_profile(
            str(self.get_parameter("camera_qos_reliability").value)
        )
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.production_mode = bool(self.get_parameter("production_mode").value)
        self.require_request_id = bool(self.get_parameter("require_request_id").value)
        self.require_operator_id = bool(self.get_parameter("require_operator_id").value)
        self.max_request_bytes = max(
            1024, int(self.get_parameter("max_request_bytes").value)
        )
        self.command_rate_limit = max(
            1, int(self.get_parameter("command_rate_limit_per_minute").value)
        )
        self.system_health_timeout = max(
            0.1, float(self.get_parameter("system_health_timeout_s").value)
        )
        audit_path = os.environ.get(
            "AIR_GROUND_GATEWAY_AUDIT_LOG",
            str(self.get_parameter("audit_log_path").value),
        )
        self.audit = AuditJournal(
            audit_path,
            max_bytes=int(self.get_parameter("audit_log_max_bytes").value),
            required=bool(self.get_parameter("audit_required").value),
        )
        if self.production_mode and self.command_enabled:
            if not control_token_is_strong(self.auth_token):
                raise RuntimeError(
                    "production command control requires a non-placeholder, "
                    "high-diversity token of at least 32 characters"
                )
            if self.cors_origin == "*":
                raise RuntimeError("production command control requires an explicit CORS origin")
            if not self.audit.available:
                raise RuntimeError("production command control requires a writable audit journal")

        self.lock = threading.RLock()
        self.commands: queue.Queue[GatewayCommand] = queue.Queue(maxsize=64)
        self.shutdown_event = threading.Event()
        self.started_at = time.monotonic()
        self.last_teleop = 0.0
        self.operator_navigation_active = False
        self.operator_nav_generation = 0
        self.operator_goal_handle = None
        self.command_windows: Dict[str, list[float]] = {}
        self.metrics = {
            "commands_accepted": 0,
            "commands_rejected": 0,
            "authentication_failures": 0,
            "rate_limit_rejections": 0,
        }
        self.latest = {
            "mission": {},
            "mavlink": {},
            "perception": {},
            "docking": {},
            "navigation": {},
            "command_mux": {},
            "optical_flow": {},
            "system": {},
            "ugv": {"pose": None, "speed_mps": 0.0, "minimum_scan_m": None},
            "ugv_control_mux": {},
            "chassis_adapter": {},
            "ugv_gateway": {},
            "paths": {
                "ugv_global": [],
                "ugv_global_for_state": "",
                "ugv_global_for_transition": None,
                "ugv_global_for_goal_status": "",
            },
        }
        self.topic_times: Dict[str, float] = {}
        self.images: Dict[str, bytes] = {}
        self.image_times: Dict[str, float] = {}
        self.image_source_times: Dict[str, float] = {}
        self.image_enqueue_times: Dict[str, float] = {}
        self.pending_images: Dict[str, tuple[Image, float]] = {}
        self.queued_cameras: set[str] = set()
        self.camera_encode_queue: queue.Queue[Optional[str]] = queue.Queue()
        self.camera_workers = [
            threading.Thread(
                target=self.camera_encode_worker,
                name=f"camera-jpeg-{index + 1}",
                daemon=True,
            )
            for index in range(self.camera_encoder_threads)
        ]
        for worker in self.camera_workers:
            worker.start()
        self.camera_subscriptions = {}
        # Isolate image reception from control/status callbacks and allow
        # lower-rate streams to run even when another image topic is ready.
        self.camera_callback_group = ReentrantCallbackGroup()
        self.dashboard_clients = 0
        self.camera_interest_until: Dict[str, float] = {}

        for topic, key in (
            ("/mission/status", "mission"),
            ("/uav/mavlink/status", "mavlink"),
            ("/uav/perception/status", "perception"),
            ("/uav/docking/status", "docking"),
            ("/uav/navigation/status", "navigation"),
            ("/uav/command_mux/status", "command_mux"),
            ("/uav/optical_flow/status", "optical_flow"),
            ("/system/health", "system"),
            ("/ugv/control_mux/status", "ugv_control_mux"),
            ("/ugv/chassis_adapter/status", "chassis_adapter"),
            ("/ugv/command_gateway/status", "ugv_gateway"),
        ):
            self.create_subscription(
                String,
                topic,
                lambda message, name=key: self.on_json(name, message),
                10,
            )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_ugv_pose, 10
        )
        self.create_subscription(Odometry, "/odometry/filtered", self.on_ugv_odom, 10)
        self.create_subscription(
            LaserScan, "/scan", self.on_ugv_scan, qos_profile_sensor_data
        )
        self.create_subscription(Path, "/plan", self.on_ugv_path, 10)

        self.uav_goal_publisher = self.create_publisher(PoseStamped, "/uav/nav/goal", 10)
        self.gimbal_publisher = self.create_publisher(Vector3, "/uav/gimbal/setpoint", 10)
        self.ugv_teleop_publisher = self.create_publisher(
            Twist, "/ugv/teleop/cmd_vel", 10
        )
        self.operator_heartbeat_publisher = self.create_publisher(
            Bool, "/ugv/operator/heartbeat", 10
        )
        self.mission_start = self.create_client(Trigger, "/air_ground_mission/start")
        self.mission_abort = self.create_client(Trigger, "/air_ground_mission/abort")
        self.mission_reset = self.create_client(Trigger, "/air_ground_mission/reset")
        self.mission_pause = self.create_client(SetBool, "/air_ground_mission/pause")
        self.uav_navigation_enable = self.create_client(SetBool, "/uav_navigation/enable")
        self.safety_estop = self.create_client(
            SetBool, "/system_supervisor/emergency_stop"
        )
        self.safety_reset = self.create_client(Trigger, "/system_supervisor/reset")
        self.navigate_action = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.command_timer = create_steady_timer(self, 0.05, self.process_commands)
        self.teleop_watchdog = create_steady_timer(
            self, 0.1, self.stop_stale_teleop
        )
        self.camera_subscription_timer = create_steady_timer(
            self, 0.5, self.update_camera_subscriptions
        )
        self.http_server = ThreadingHTTPServer(
            (self.bind_address, self.port), self._handler_class()
        )
        self.http_server.daemon_threads = True
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            name="air-ground-http",
            daemon=True,
        )
        self.http_thread.start()
        self.get_logger().info(
            f"Dashboard gateway listening on http://{self.bind_address}:{self.port}; "
            f"commands_enabled={self.command_enabled}, token_required={bool(self.auth_token)}"
        )

    def _handler_class(self):
        gateway = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "AirGroundGateway/2.0"

            def log_message(self, format_string, *args):
                gateway.get_logger().debug(format_string % args)

            def _headers(self, status: int, content_type: str, length: Optional[int] = None):
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                origin = self.headers.get("Origin", "")
                if gateway.cors_origin == "*":
                    self.send_header("Access-Control-Allow-Origin", "*")
                elif origin and hmac.compare_digest(origin, gateway.cors_origin):
                    self.send_header("Access-Control-Allow-Origin", origin)
                    self.send_header("Vary", "Origin")
                self.send_header(
                    "Access-Control-Allow-Headers",
                    "Content-Type, Authorization, X-Control-Token, X-Request-ID, X-Operator-ID",
                )
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header("Referrer-Policy", "no-referrer")
                if length is not None:
                    self.send_header("Content-Length", str(length))
                self.end_headers()

            def _json(self, status: int, body: dict):
                encoded = json.dumps(body, ensure_ascii=False).encode("utf-8")
                try:
                    self._headers(status, "application/json; charset=utf-8", len(encoded))
                    self.wfile.write(encoded)
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    # Browsers routinely cancel superseded status / image
                    # requests; that is not a gateway fault.
                    pass

            def _authorized(self) -> bool:
                if not gateway.command_enabled:
                    return False
                if not gateway.auth_token:
                    return True
                bearer = self.headers.get("Authorization", "")
                token = self.headers.get("X-Control-Token", "")
                expected_bearer = f"Bearer {gateway.auth_token}"
                return hmac.compare_digest(bearer, expected_bearer) or hmac.compare_digest(
                    token, gateway.auth_token
                )

            def do_OPTIONS(self):
                self._headers(HTTPStatus.NO_CONTENT, "text/plain", 0)

            def do_GET(self):
                path = urlparse(self.path).path
                if path == "/api/health":
                    self._json(HTTPStatus.OK, gateway.health_snapshot())
                    return
                if path == "/api/status":
                    self._json(HTTPStatus.OK, gateway.status_snapshot())
                    return
                if path == "/api/ready":
                    readiness = gateway.readiness_snapshot()
                    self._json(
                        HTTPStatus.OK if readiness["ready"] else HTTPStatus.SERVICE_UNAVAILABLE,
                        readiness,
                    )
                    return
                if path == "/api/metrics":
                    encoded = gateway.metrics_snapshot().encode("utf-8")
                    self._headers(HTTPStatus.OK, "text/plain; version=0.0.4", len(encoded))
                    self.wfile.write(encoded)
                    return
                if path == "/api/events":
                    self._serve_events()
                    return
                if path.startswith("/api/camera/") and path.endswith(".jpg"):
                    name = path.rsplit("/", 1)[-1][:-4]
                    if not gateway.request_camera(name):
                        self._json(
                            HTTPStatus.NOT_FOUND,
                            {"error": "unknown_camera", "camera": name},
                        )
                        return
                    with gateway.lock:
                        image = gateway.images.get(name)
                        image_time = gateway.image_times.get(name)
                    if image is None or not camera_frame_is_fresh(
                        image_time, time.monotonic(), gateway.camera_stale_after
                    ):
                        self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": "camera_not_ready", "camera": name})
                        return
                    try:
                        self._headers(HTTPStatus.OK, "image/jpeg", len(image))
                        self.wfile.write(image)
                    except (BrokenPipeError, ConnectionResetError, TimeoutError):
                        pass
                    return
                self._json(HTTPStatus.NOT_FOUND, {"error": "not_found"})

            def _serve_events(self):
                self._headers(HTTPStatus.OK, "text/event-stream; charset=utf-8")
                gateway.register_dashboard_client()
                try:
                    while not gateway.shutdown_event.is_set():
                        payload = json.dumps(gateway.status_snapshot(), ensure_ascii=False)
                        self.wfile.write(f"event: status\ndata: {payload}\n\n".encode("utf-8"))
                        self.wfile.flush()
                        gateway.shutdown_event.wait(0.5)
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    pass
                finally:
                    gateway.unregister_dashboard_client()

            def do_POST(self):
                remote = trusted_client_address(
                    str(self.client_address[0]),
                    self.headers.get("X-Forwarded-For", ""),
                    trust_proxy_headers=gateway.trust_proxy_headers,
                )
                device_id = trusted_proxy_identity(
                    str(self.client_address[0]),
                    self.headers.get("X-Operator-Device", ""),
                    trust_proxy_headers=gateway.trust_proxy_headers,
                )
                request_id = self.headers.get("X-Request-ID", "").strip()
                operator_id = self.headers.get("X-Operator-ID", "").strip()
                if not request_id:
                    request_id = str(uuid.uuid4())
                if not self._authorized():
                    code = HTTPStatus.FORBIDDEN if not gateway.command_enabled else HTTPStatus.UNAUTHORIZED
                    gateway.metrics["authentication_failures"] += 1
                    gateway.metrics["commands_rejected"] += 1
                    gateway.audit_command(remote, operator_id, request_id, "unknown", False, "unauthorized", device_id=device_id)
                    self._json(code, {"error": "commands_disabled_or_unauthorized", "request_id": request_id})
                    return
                if gateway.require_request_id and not self.headers.get("X-Request-ID", "").strip():
                    gateway.metrics["commands_rejected"] += 1
                    gateway.audit_command(remote, operator_id, request_id, "unknown", False, "request_id_required", device_id=device_id)
                    self._json(HTTPStatus.BAD_REQUEST, {"error": "request_id_required", "request_id": request_id})
                    return
                if gateway.require_operator_id and not operator_id:
                    gateway.metrics["commands_rejected"] += 1
                    gateway.audit_command(remote, operator_id, request_id, "unknown", False, "operator_id_required", device_id=device_id)
                    self._json(HTTPStatus.BAD_REQUEST, {"error": "operator_id_required", "request_id": request_id})
                    return
                if not gateway.consume_rate_limit(remote):
                    gateway.metrics["rate_limit_rejections"] += 1
                    gateway.metrics["commands_rejected"] += 1
                    gateway.audit_command(remote, operator_id, request_id, "unknown", False, "rate_limited", device_id=device_id)
                    self._json(HTTPStatus.TOO_MANY_REQUESTS, {"error": "rate_limited", "request_id": request_id})
                    return
                path = urlparse(self.path).path
                try:
                    length = int(self.headers.get("Content-Length", "0") or 0)
                except ValueError:
                    length = -1
                if length < 0 or length > gateway.max_request_bytes:
                    gateway.metrics["commands_rejected"] += 1
                    gateway.audit_command(remote, operator_id, request_id, "unknown", False, "request_too_large", device_id=device_id)
                    self._json(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, {"error": "request_too_large", "request_id": request_id})
                    return
                payload = {}
                if length:
                    try:
                        payload = json.loads(self.rfile.read(length).decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        gateway.metrics["commands_rejected"] += 1
                        gateway.audit_command(remote, operator_id, request_id, "unknown", False, "invalid_json", device_id=device_id)
                        self._json(HTTPStatus.BAD_REQUEST, {"error": "invalid_json", "request_id": request_id})
                        return
                    if not isinstance(payload, dict):
                        self._json(HTTPStatus.BAD_REQUEST, {"error": "json_object_required", "request_id": request_id})
                        return
                command_name = {
                    "/api/mission/start": "mission_start",
                    "/api/mission/pause": "mission_pause",
                    "/api/mission/resume": "mission_resume",
                    "/api/mission/abort": "mission_abort",
                    "/api/mission/reset": "mission_reset",
                    "/api/uav/goal": "uav_goal",
                    "/api/ugv/goal": "ugv_goal",
                    "/api/ugv/teleop": "ugv_teleop",
                    "/api/gimbal": "gimbal",
                    "/api/sim/pause": "sim_pause",
                    "/api/sim/resume": "sim_resume",
                    "/api/sim/reset": "sim_reset",
                    "/api/safety/emergency-stop": "safety_estop",
                    "/api/safety/reset": "safety_reset",
                }.get(path)
                if command_name is None:
                    gateway.metrics["commands_rejected"] += 1
                    gateway.audit_command(remote, operator_id, request_id, "unknown", False, "unknown_command", device_id=device_id)
                    self._json(HTTPStatus.NOT_FOUND, {"error": "unknown_command", "request_id": request_id})
                    return
                allowed, denial = gateway.command_allowed(command_name)
                if not allowed:
                    result = {"accepted": False, "error": denial, "request_id": request_id}
                    gateway.metrics["commands_rejected"] += 1
                    gateway.audit_command(remote, operator_id, request_id, command_name, False, denial, payload, device_id=device_id)
                    self._json(HTTPStatus.SERVICE_UNAVAILABLE, result)
                    return
                result = gateway.enqueue_command(
                    command_name, payload, request_id=request_id, operator_id=operator_id
                )
                result["request_id"] = request_id
                accepted = bool(result.get("accepted"))
                gateway.metrics["commands_accepted" if accepted else "commands_rejected"] += 1
                gateway.audit_command(
                    remote,
                    operator_id,
                    request_id,
                    command_name,
                    accepted,
                    str(result.get("status", result.get("error", "unknown"))),
                    payload,
                    device_id=device_id,
                )
                self._json(
                    HTTPStatus.ACCEPTED if accepted else HTTPStatus.CONFLICT,
                    result,
                )

        return Handler

    def register_dashboard_client(self) -> None:
        with self.lock:
            self.dashboard_clients += 1

    def unregister_dashboard_client(self) -> None:
        with self.lock:
            self.dashboard_clients = max(0, self.dashboard_clients - 1)

    def request_camera(self, key: str) -> bool:
        if key not in CAMERA_TOPICS:
            return False
        now = time.monotonic()
        with self.lock:
            self.camera_interest_until[key] = now + self.camera_stream_idle_timeout
        return True

    def update_camera_subscriptions(self) -> None:
        now = time.monotonic()
        with self.lock:
            desired = requested_camera_keys(
                self.dashboard_clients, self.camera_interest_until, now
            )
            active = set(self.camera_subscriptions)

        for key in sorted(active - desired):
            with self.lock:
                subscription = self.camera_subscriptions.pop(key, None)
            if subscription is not None:
                self.destroy_subscription(subscription)

        for key in sorted(desired - active):
            subscription = self.create_subscription(
                Image,
                CAMERA_TOPICS[key],
                lambda message, name=key: self.on_image(name, message),
                self.camera_qos,
                callback_group=self.camera_callback_group,
            )
            with self.lock:
                self.camera_subscriptions[key] = subscription

    def on_json(self, key: str, message: String) -> None:
        with self.lock:
            self.latest[key] = _json_message(message)
            self.topic_times[key] = time.monotonic()

    def on_ugv_pose(self, message: PoseWithCovarianceStamped) -> None:
        position = message.pose.pose.position
        yaw = _yaw_from_quaternion(message.pose.pose.orientation)
        with self.lock:
            self.latest["ugv"]["pose"] = [
                round(position.x, 3),
                round(position.y, 3),
                round(yaw, 3),
            ]
            self.topic_times["ugv_pose"] = time.monotonic()

    def on_ugv_odom(self, message: Odometry) -> None:
        with self.lock:
            self.latest["ugv"]["speed_mps"] = round(
                math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y), 3
            )
            self.topic_times["ugv_odom"] = time.monotonic()

    def on_ugv_scan(self, message: LaserScan) -> None:
        ranges = [value for value in message.ranges if math.isfinite(value) and value > 0.0]
        with self.lock:
            self.latest["ugv"]["minimum_scan_m"] = None if not ranges else round(min(ranges), 3)
            self.topic_times["ugv_scan"] = time.monotonic()

    def on_ugv_path(self, message: Path) -> None:
        # A bounded path payload keeps SSE frames light enough for tablets.
        poses = message.poses
        stride = max(len(poses) // 80, 1)
        points = [
            [round(pose.pose.position.x, 2), round(pose.pose.position.y, 2)]
            for pose in poses[::stride]
        ]
        with self.lock:
            mission = self.latest.get("mission") or {}
            self.latest["paths"]["ugv_global"] = points
            self.latest["paths"]["ugv_global_for_state"] = str(
                mission.get("state", "")
            )
            self.latest["paths"]["ugv_global_for_transition"] = mission.get(
                "transitions"
            )
            self.latest["paths"]["ugv_global_for_goal_status"] = str(
                mission.get("ugv_goal_status", "")
            )
            self.topic_times["ugv_path"] = time.monotonic()

    def on_image(self, key: str, message: Image) -> None:
        now = time.monotonic()
        should_enqueue = False
        with self.lock:
            self.image_source_times[key] = now
            if now - self.image_enqueue_times.get(key, 0.0) < self.camera_interval:
                return
            self.image_enqueue_times[key] = now
            self.pending_images[key] = (message, now)
            if key not in self.queued_cameras:
                self.queued_cameras.add(key)
                should_enqueue = True
        if should_enqueue:
            self.camera_encode_queue.put_nowait(key)

    def camera_encode_worker(self) -> None:
        """Encode only the newest pending frame without blocking ROS callbacks."""

        bridge = CvBridge()
        while not self.shutdown_event.is_set():
            try:
                key = self.camera_encode_queue.get(timeout=0.25)
            except queue.Empty:
                continue
            if key is None:
                self.camera_encode_queue.task_done()
                break
            with self.lock:
                pending = self.pending_images.pop(key, None)
            if pending is None:
                with self.lock:
                    self.queued_cameras.discard(key)
                self.camera_encode_queue.task_done()
                continue
            message, source_time = pending
            self.encode_camera_frame(bridge, key, message, source_time)
            requeue = False
            with self.lock:
                if key in self.pending_images:
                    requeue = True
                else:
                    self.queued_cameras.discard(key)
            if requeue:
                self.camera_encode_queue.put_nowait(key)
            self.camera_encode_queue.task_done()

    def encode_camera_frame(
        self, bridge: CvBridge, key: str, message: Image, source_time: float
    ) -> None:
        try:
            frame = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            if frame.shape[1] > 800:
                scale = 800.0 / frame.shape[1]
                frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            success, encoded = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            )
        except Exception as error:
            self.get_logger().warning(f"Camera {key} conversion failed: {error}")
            return
        if success:
            encoded_at = time.monotonic()
            with self.lock:
                self.images[key] = encoded.tobytes()
                self.image_times[key] = encoded_at
                self.topic_times[f"camera_{key}"] = source_time

    def health_snapshot(self) -> dict:
        return {
            "ok": not (self.audit.required and not self.audit.available),
            "schema_version": "1.0",
            "service": "air-ground-dashboard-gateway",
            "uptime_s": round(time.monotonic() - self.started_at, 1),
            "command_enabled": self.command_enabled,
            "token_required": bool(self.auth_token),
            "production_mode": self.production_mode,
            "audit_available": self.audit.available,
            "request_id_required": self.require_request_id,
            "operator_identity_required": self.require_operator_id,
        }

    def readiness_snapshot(self) -> dict:
        now = time.monotonic()
        with self.lock:
            system = dict(self.latest.get("system") or {})
            timestamp = self.topic_times.get("system")
        age = None if timestamp is None else now - timestamp
        ready = (
            age is not None
            and age <= self.system_health_timeout
            and bool(system.get("ready", False))
            and not bool(system.get("emergency_stop", False))
            and not (self.audit.required and not self.audit.available)
        )
        return {
            "ready": ready,
            "system_state": system.get("state", "UNAVAILABLE"),
            "system_health_age_s": None if age is None else round(age, 3),
            "emergency_stop": bool(system.get("emergency_stop", False)),
        }

    def metrics_snapshot(self) -> str:
        readiness = self.readiness_snapshot()
        lines = [
            "# TYPE air_ground_gateway_ready gauge",
            f"air_ground_gateway_ready {1 if readiness['ready'] else 0}",
            "# TYPE air_ground_gateway_uptime_seconds gauge",
            f"air_ground_gateway_uptime_seconds {time.monotonic() - self.started_at:.3f}",
        ]
        for key, value in sorted(self.metrics.items()):
            lines.extend(
                [
                    f"# TYPE air_ground_gateway_{key}_total counter",
                    f"air_ground_gateway_{key}_total {int(value)}",
                ]
            )
        return "\n".join(lines) + "\n"

    def consume_rate_limit(self, remote: str) -> bool:
        now = time.monotonic()
        with self.lock:
            window = self.command_windows.setdefault(remote, [])
            window[:] = [timestamp for timestamp in window if now - timestamp < 60.0]
            if len(window) >= self.command_rate_limit:
                return False
            window.append(now)
        return True

    def audit_command(
        self,
        remote: str,
        operator_id: str,
        request_id: str,
        command: str,
        accepted: bool,
        outcome: str,
        payload: Optional[dict] = None,
        *,
        device_id: str = "",
    ) -> None:
        self.audit.write(
            {
                "event": "control_command",
                "remote": remote,
                "operator_id": operator_id or "anonymous",
                "operator_device": device_id or "unavailable",
                "request_id": request_id,
                "command": command,
                "accepted": bool(accepted),
                "outcome": outcome,
                "payload": payload or {},
            }
        )

    def command_allowed(self, name: str) -> tuple[bool, str]:
        always_allowed = {
            "mission_abort",
            "mission_reset",
            "safety_estop",
            "safety_reset",
            "sim_pause",
            "sim_reset",
        }
        if name in always_allowed:
            return True, ""
        if not production_motion_plan_allows(
            production_mode=self.production_mode,
            command=name,
            mission_status=self.latest.get("mission"),
        ):
            return False, "commissioned_mission_plan_required"
        readiness = self.readiness_snapshot()
        if readiness["emergency_stop"]:
            return False, "system_emergency_stop_active"
        if not readiness["ready"]:
            return False, "system_not_ready"
        return True, ""

    def status_snapshot(self) -> dict:
        now = time.monotonic()
        with self.lock:
            copied = json.loads(json.dumps(self.latest))
            topic_ages = {
                key: round(now - timestamp, 2) for key, timestamp in self.topic_times.items()
            }
            path_timestamp = self.topic_times.get("ugv_path")
            copied["paths"]["ugv_global_age_s"] = (
                None
                if path_timestamp is None
                else round(max(0.0, now - path_timestamp), 2)
            )
            active_cameras = set(self.camera_subscriptions)
            cameras = {}
            for key in CAMERA_TOPICS:
                timestamp = self.image_times.get(key)
                age = None if timestamp is None else max(0.0, now - timestamp)
                source_timestamp = self.image_source_times.get(key)
                source_age = (
                    None
                    if source_timestamp is None
                    else max(0.0, now - source_timestamp)
                )
                cameras[key] = {
                    "active": key in active_cameras,
                    "ready": bool(
                        key in active_cameras
                        and camera_frame_is_fresh(
                            timestamp, now, self.camera_stale_after
                        )
                    ),
                    "age_s": None if age is None else round(age, 2),
                    "source_age_s": (
                        None if source_age is None else round(source_age, 2)
                    ),
                }
        copied.update(
            {
                "gateway": self.health_snapshot(),
                "readiness": self.readiness_snapshot(),
                "server_time_ms": int(time.time() * 1000),
                "topic_ages_s": topic_ages,
                "cameras": cameras,
                "world": {
                    "name": self.simulation_world,
                    "bounds": list(self.world_bounds),
                    "no_fly_zones": self.no_fly_zones,
                    "height_limit_zones": self.height_limit_zones,
                },
            }
        )
        return copied

    def enqueue_command(
        self,
        name: str,
        payload: dict,
        *,
        request_id: str = "",
        operator_id: str = "",
    ) -> dict:
        command = GatewayCommand(
            name=name,
            payload=payload if isinstance(payload, dict) else {},
            request_id=request_id,
            operator_id=operator_id,
        )
        try:
            self.commands.put_nowait(command)
        except queue.Full:
            return {"accepted": False, "error": "command_queue_full"}
        if not command.completed.wait(2.0):
            return {"accepted": True, "status": "queued", "command": name}
        return command.result

    def _service_command(self, client, request) -> dict:
        if not client.service_is_ready():
            return {"accepted": False, "error": "ros_service_not_ready"}
        client.call_async(request)
        return {"accepted": True, "status": "requested"}

    def _require_number(self, payload: dict, key: str, minimum: float, maximum: float) -> float:
        value = float(payload[key])
        if not math.isfinite(value) or not minimum <= value <= maximum:
            raise ValueError(f"{key} outside [{minimum}, {maximum}]")
        return value

    def process_commands(self) -> None:
        for _ in range(10):
            try:
                command = self.commands.get_nowait()
            except queue.Empty:
                return
            try:
                command.result = self._execute(command.name, command.payload)
            except (KeyError, TypeError, ValueError) as error:
                command.result = {"accepted": False, "error": str(error)}
            except Exception as error:
                self.get_logger().error(f"Gateway command {command.name} failed: {error}")
                command.result = {"accepted": False, "error": "internal_command_error"}
            finally:
                command.completed.set()
                self.commands.task_done()

    def _execute(self, name: str, payload: dict) -> dict:
        if name == "mission_start":
            self._cancel_operator_control()
            return self._service_command(self.mission_start, Trigger.Request())
        if name == "mission_abort":
            return self._service_command(self.mission_abort, Trigger.Request())
        if name == "mission_reset":
            return self._service_command(self.mission_reset, Trigger.Request())
        if name in ("mission_pause", "mission_resume"):
            request = SetBool.Request()
            request.data = name == "mission_pause"
            return self._service_command(self.mission_pause, request)
        if name == "safety_estop":
            request = SetBool.Request()
            request.data = True
            self._cancel_operator_control()
            return self._service_command(self.safety_estop, request)
        if name == "safety_reset":
            return self._service_command(self.safety_reset, Trigger.Request())
        if name == "uav_goal":
            mission = self.latest.get("mission") or {}
            if bool(mission.get("active", False)) and not bool(mission.get("paused", False)):
                return {"accepted": False, "error": "pause_active_mission_before_manual_goal"}
            x = self._require_number(
                payload, "x", self.world_bounds[0], self.world_bounds[1]
            )
            y = self._require_number(
                payload, "y", self.world_bounds[2], self.world_bounds[3]
            )
            z = self._require_number(
                payload,
                "z",
                self.manual_uav_min_altitude,
                self.manual_uav_max_altitude,
            )
            yaw = self._require_number(payload, "yaw", -math.pi, math.pi) if "yaw" in payload else 0.0
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = "uav_odom"
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = x, y, z
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.orientation.w = math.cos(yaw / 2.0)
            self.uav_goal_publisher.publish(goal)
            enable = SetBool.Request()
            enable.data = True
            self._service_command(self.uav_navigation_enable, enable)
            return {"accepted": True, "status": "goal_published", "goal": [x, y, z, yaw]}
        if name == "ugv_goal":
            mission = self.latest.get("mission") or {}
            if bool(mission.get("active", False)) and not bool(mission.get("paused", False)):
                return {"accepted": False, "error": "pause_active_mission_before_manual_goal"}
            x = self._require_number(
                payload, "x", self.world_bounds[0], self.world_bounds[1]
            )
            y = self._require_number(
                payload, "y", self.world_bounds[2], self.world_bounds[3]
            )
            yaw = self._require_number(payload, "yaw", -math.pi, math.pi) if "yaw" in payload else 0.0
            if not self.navigate_action.server_is_ready():
                return {"accepted": False, "error": "nav2_action_not_ready"}
            goal = NavigateToPose.Goal()
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.header.frame_id = "map"
            goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.operator_nav_generation += 1
            generation = self.operator_nav_generation
            self.operator_navigation_active = True
            future = self.navigate_action.send_goal_async(goal)
            future.add_done_callback(
                lambda completed, goal_generation=generation: self._on_operator_goal_response(
                    completed, goal_generation
                )
            )
            return {"accepted": True, "status": "goal_sent", "goal": [x, y, yaw]}
        if name == "gimbal":
            yaw = self._require_number(payload, "yaw", -2.967, 2.967)
            pitch = self._require_number(payload, "pitch", -0.436, 1.5708)
            setpoint = Vector3()
            setpoint.x, setpoint.y = yaw, pitch
            self.gimbal_publisher.publish(setpoint)
            return {"accepted": True, "status": "setpoint_published", "setpoint": [yaw, pitch]}
        if name == "ugv_teleop":
            mission = self.latest.get("mission") or {}
            if bool(mission.get("active", False)) and not bool(mission.get("paused", False)):
                return {"accepted": False, "error": "pause_active_mission_before_teleop"}
            linear = self._require_number(payload, "linear", -0.5, 0.5)
            angular = self._require_number(payload, "angular", -0.7, 0.7)
            command = Twist()
            command.linear.x, command.angular.z = linear, angular
            self.ugv_teleop_publisher.publish(command)
            self.last_teleop = time.monotonic()
            return {"accepted": True, "status": "teleop_published"}
        if name in ("sim_pause", "sim_resume", "sim_reset"):
            if name == "sim_reset":
                self._cancel_operator_control()
            return self._simulation_command(name)
        return {"accepted": False, "error": "unsupported_command"}

    def _on_operator_goal_response(self, future, generation: int) -> None:
        try:
            handle = future.result()
        except Exception as error:
            if generation == self.operator_nav_generation:
                self.operator_navigation_active = False
                self.get_logger().warning(f"Operator Nav2 goal failed: {error}")
            return
        if generation != self.operator_nav_generation:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.operator_navigation_active = False
            return
        self.operator_goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(
            lambda _completed, goal_generation=generation: self._on_operator_goal_result(
                goal_generation
            )
        )

    def _on_operator_goal_result(self, generation: int) -> None:
        if generation == self.operator_nav_generation:
            self.operator_navigation_active = False
            self.operator_goal_handle = None

    def _cancel_operator_control(self) -> None:
        self.operator_nav_generation += 1
        self.operator_navigation_active = False
        goal_handle = self.operator_goal_handle
        self.operator_goal_handle = None
        self.last_teleop = 0.0
        try:
            if goal_handle is not None:
                goal_handle.cancel_goal_async()
            self.ugv_teleop_publisher.publish(Twist())
            heartbeat = Bool()
            heartbeat.data = False
            self.operator_heartbeat_publisher.publish(heartbeat)
        except RCLError:
            # SIGINT may invalidate the ROS context between rclpy.ok() and a
            # best-effort final stop publication. Runtime watchdogs and the
            # independent supervisor remain the authoritative stop paths.
            pass

    def _simulation_command(self, name: str) -> dict:
        if not self.simulation_control_enabled:
            return {"accepted": False, "error": "simulation_control_disabled"}
        request = {
            "sim_pause": "pause: true",
            "sim_resume": "pause: false",
            "sim_reset": "reset: {all: true}",
        }[name]
        completed = subprocess.run(
            [
                "gz",
                "service",
                "-s",
                f"/world/{self.simulation_world}/control",
                "--reqtype",
                "gz.msgs.WorldControl",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "1500",
                "--req",
                request,
            ],
            capture_output=True,
            text=True,
            timeout=3.0,
            check=False,
        )
        return {
            "accepted": completed.returncode == 0,
            "status": name,
            "returncode": completed.returncode,
            "message": (completed.stdout or completed.stderr)[-500:],
        }

    def stop_stale_teleop(self) -> None:
        now = time.monotonic()
        if self.last_teleop and now - self.last_teleop > 0.35:
            self.ugv_teleop_publisher.publish(Twist())
            self.last_teleop = 0.0
        heartbeat = Bool()
        heartbeat.data = bool(self.operator_navigation_active or self.last_teleop)
        system = self.latest.get("system") or {}
        if bool(system.get("emergency_stop", False)):
            heartbeat.data = False
            self.operator_navigation_active = False
            self.ugv_teleop_publisher.publish(Twist())
            self.last_teleop = 0.0
        self.operator_heartbeat_publisher.publish(heartbeat)

    def destroy_node(self):
        self.shutdown_event.set()
        for _ in self.camera_workers:
            self.camera_encode_queue.put_nowait(None)
        for worker in self.camera_workers:
            worker.join(timeout=1.0)
        if rclpy.ok():
            self._cancel_operator_control()
        self.http_server.shutdown()
        self.http_server.server_close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardGateway()
    executor = MultiThreadedExecutor(num_threads=node.executor_threads)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
