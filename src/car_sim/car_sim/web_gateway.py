"""Minimal UGV web gateway: static teleop page, JSON status API, camera JPEG.

Trimmed from the air_ground_sim dashboard gateway to the UGV teleop subset:
no missions, no UAV, no Nav2, no audit/auth machinery. Binds to localhost by
default and exposes:

  GET  /                 static teleop page (car_sim/web/index.html)
  GET  /api/health       liveness probe
  GET  /api/status       mux/gateway status + /odom pose + camera state
  POST /api/ugv/teleop   {"linear": m/s, "angular": rad/s} operator command
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
import threading
import time
from typing import Optional, Tuple
from urllib.parse import urlparse

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

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
        self.declare_parameter("avoidance_node_name", "/avoidance_node")
        self.declare_parameter("mux_node_name", "/ugv_control_mux")

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
                if path == "/api/camera.jpg":
                    gateway._serve_camera(self, rear=False)
                    return
                if path == "/api/camera_rear.jpg":
                    gateway._serve_camera(self, rear=True)
                    return
                self._json(HTTPStatus.NOT_FOUND, {"error": "not_found"})

            def do_POST(self):
                path = urlparse(self.path).path
                if path not in ("/api/ugv/teleop", "/api/ugv/cruise"):
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

    # ---------------- snapshots ----------------
    def health_snapshot(self) -> dict:
        return {
            "ok": True,
            "schema_version": "1.0",
            "service": "car-web-gateway",
            "uptime_s": round(time.monotonic() - self.started_at, 1),
            "camera_backend": _CAMERA_AVAILABLE,
        }

    def status_snapshot(self) -> dict:
        now = time.monotonic()
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
            "battery_voltage": None,  # reserved for the real motor driver
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
