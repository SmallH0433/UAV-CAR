"""Read-only HTTP bridge for flight state shown by the OV9281 console."""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, PositionTarget, RCIn, State, VfrHud
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Range


LANDED_STATES = {
    ExtendedState.LANDED_STATE_UNDEFINED: "UNKNOWN",
    ExtendedState.LANDED_STATE_ON_GROUND: "ON_GROUND",
    ExtendedState.LANDED_STATE_IN_AIR: "IN_AIR",
    ExtendedState.LANDED_STATE_TAKEOFF: "TAKEOFF",
    ExtendedState.LANDED_STATE_LANDING: "LANDING",
}


def axis_direction(value: Optional[float], positive: str, negative: str, deadband: float) -> str:
    if value is None:
        return "UNKNOWN"
    if value > deadband:
        return positive
    if value < -deadband:
        return negative
    return "HOLD"


def normalize_executor_status(payload: dict) -> dict:
    """Expose v2 actions through the existing read-only console contract."""
    if payload.get("node") != "ACTION_EXECUTOR_ROS2":
        return payload
    result = dict(payload)
    active = payload.get("state") == "RUNNING" and payload.get("output_enabled") is True
    result.update(
        follow_active=active and payload.get("follow_active", payload.get("action") == "FOLLOW"),
        landing_active=active and payload.get("landing_active", payload.get("action") in {"LAND", "PRECISION_LAND"}),
        landing_requested=payload.get("landing_switch_state") == "HIGH",
        control_owner="ACTION_EXECUTOR_V2" if active else "HOLD",
        mode_gate=payload.get("detail", "UNKNOWN"),
        executor_version="v2",
    )
    return result


def command_to_body_flu(command: dict, quaternion: Optional[tuple]) -> Optional[dict]:
    """ROS MAVROS input: LOCAL frames use ENU; BODY frames use FLU."""
    v = command.get("velocity", {})
    values = [v.get(axis) for axis in ("x", "y", "z")]
    if any(not isinstance(x, (int, float)) or not math.isfinite(x) for x in values):
        return None
    if command.get("type_mask", 0) & (8 | 16 | 32):
        return None  # Not a complete velocity command; never invent masked axes.
    frame = command.get("coordinate_frame")
    if frame in (8, 9):
        return dict(zip(("x", "y", "z"), values))
    if frame not in (1, 7) or quaternion is None:
        return None
    if len(quaternion) != 4 or not all(math.isfinite(x) for x in quaternion):
        return None
    norm = math.sqrt(sum(x*x for x in quaternion))
    if norm < 1e-6:
        return None
    x, y, z, w = (x/norm for x in quaternion)
    # Transpose of body FLU -> local ENU rotation.
    east, north, up = values
    return {
        "x": (1-2*(y*y+z*z))*east + 2*(x*y+w*z)*north + 2*(x*z-w*y)*up,
        "y": 2*(x*y-w*z)*east + (1-2*(x*x+z*z))*north + 2*(y*z+w*x)*up,
        "z": 2*(x*z+w*y)*east + 2*(y*z-w*x)*north + (1-2*(x*x+y*y))*up,
    }


def command_yaw_rate(command: dict) -> Optional[float]:
    """Return only an enabled, finite outgoing yaw-rate setpoint (ROS rad/s)."""
    if int(command.get("type_mask", 0)) & PositionTarget.IGNORE_YAW_RATE:
        return None
    value = command.get("yaw_rate")
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return None
    return float(value)


class FlightStatusState:
    def __init__(self, velocity_deadband_mps: float) -> None:
        self.lock = threading.Lock()
        self.velocity_deadband_mps = velocity_deadband_mps
        self.vehicle: dict = {
            "connected": False,
            "armed": False,
            "mode": "UNKNOWN",
            "system_status": 0,
        }
        self.vehicle_received_s: Optional[float] = None
        self.velocity: Optional[dict[str, float]] = None
        self.velocity_frame = ""
        self.velocity_received_s: Optional[float] = None
        self.extended_state = "UNKNOWN"
        self.extended_received_s: Optional[float] = None
        self.executor_status: dict = {}
        self.executor_received_s: Optional[float] = None
        self.command: Optional[dict] = None
        self.command_received_s: Optional[float] = None
        self.pose_quaternion: Optional[tuple] = None
        self.pose_received_s: Optional[float] = None
        self.pose_altitude_m: Optional[float] = None
        self.local_vertical_speed_mps: Optional[float] = None
        self.local_velocity_received_s: Optional[float] = None
        self.rangefinder_m: Optional[float] = None
        self.rangefinder_received_s: Optional[float] = None
        self.rc3_pwm: Optional[int] = None
        self.rc_received_s: Optional[float] = None
        self.motor_throttle_pct: Optional[float] = None
        self.vfr_altitude_m: Optional[float] = None
        self.vfr_received_s: Optional[float] = None
        self.history = deque(maxlen=1800)  # 30 minutes at 1 Hz, in memory only.

    def history_snapshot(self) -> dict:
        with self.lock:
            return {"sample_period_s": 1, "retention_minutes": 30,
                    "samples": list(self.history)}

    def record_history(self) -> None:
        current = self.snapshot()
        telemetry = current["telemetry"]
        sample = {"t_ms": int(time.time() * 1000),
                  "mode": current["mode"], "connected": current["flight_controller_connected"],
                  "armed": current["armed"], **telemetry}
        with self.lock:
            self.history.append(sample)

    def snapshot(self) -> dict:
        now_s = time.monotonic()
        with self.lock:
            vehicle = dict(self.vehicle)
            velocity = None if self.velocity is None else dict(self.velocity)
            executor = dict(self.executor_status)
            vehicle_age = None if self.vehicle_received_s is None else max(0.0, now_s - self.vehicle_received_s)
            velocity_age = None if self.velocity_received_s is None else max(0.0, now_s - self.velocity_received_s)
            extended_age = None if self.extended_received_s is None else max(0.0, now_s - self.extended_received_s)
            executor_age = None if self.executor_received_s is None else max(0.0, now_s - self.executor_received_s)
            extended_state = self.extended_state
            velocity_frame = self.velocity_frame
            command = self.command
            command_age = None if self.command_received_s is None else max(0.0, now_s - self.command_received_s)
            pose_age = None if self.pose_received_s is None else max(0.0, now_s - self.pose_received_s)
            quaternion = self.pose_quaternion if pose_age is not None and pose_age <= 0.5 else None
            pose_altitude = self.pose_altitude_m
            local_speed = self.local_vertical_speed_mps
            local_age = None if self.local_velocity_received_s is None else max(0.0, now_s - self.local_velocity_received_s)
            rangefinder = self.rangefinder_m
            range_age = None if self.rangefinder_received_s is None else max(0.0, now_s - self.rangefinder_received_s)
            rc3 = self.rc3_pwm
            rc_age = None if self.rc_received_s is None else max(0.0, now_s - self.rc_received_s)
            motor_throttle = self.motor_throttle_pct
            vfr_altitude = self.vfr_altitude_m
            vfr_age = None if self.vfr_received_s is None else max(0.0, now_s - self.vfr_received_s)
        velocity_fresh = velocity is not None and velocity_age is not None and velocity_age <= 1.0
        vx = velocity["x"] if velocity_fresh else None
        vy = velocity["y"] if velocity_fresh else None
        vz = velocity["z"] if velocity_fresh else None
        horizontal = axis_direction(vx, "FORWARD", "BACKWARD", self.velocity_deadband_mps)
        lateral = axis_direction(vy, "LEFT", "RIGHT", self.velocity_deadband_mps)
        vertical = axis_direction(vz, "UP", "DOWN", self.velocity_deadband_mps)
        connected = bool(vehicle.get("connected", False)) and vehicle_age is not None and vehicle_age <= 1.0
        command_fresh = connected and vehicle.get("mode") == "GUIDED" and command_age is not None and command_age <= 0.5
        command_body = command_to_body_flu(command, quaternion) if command_fresh and command else None
        yaw_rate = command_yaw_rate(command) if command_fresh and command else None
        command_state = "READY" if command_body is not None else ("FRAME_OR_POSE_UNAVAILABLE" if command_fresh else "NO_FRESH_COMMAND")
        telemetry = {
            "estimated_altitude_m": pose_altitude if connected and pose_age is not None and pose_age <= 1.0 else None,
            "rangefinder_m": rangefinder if connected and range_age is not None and range_age <= 1.0 else None,
            "vertical_speed_mps": local_speed if connected and local_age is not None and local_age <= 1.0 else None,
            "commanded_vertical_speed_mps": (float(command["velocity"]["z"])
                if command_fresh and command and not (command["type_mask"] & 32)
                and math.isfinite(float(command["velocity"]["z"])) else None),
            "rc3_pwm": rc3 if connected and rc_age is not None and rc_age <= 1.0 else None,
            "motor_throttle_pct": motor_throttle if connected and vfr_age is not None and vfr_age <= 1.0 else None,
            "vfr_altitude_msl_m": vfr_altitude if connected and vfr_age is not None and vfr_age <= 1.0 else None,
        }
        return {
            "available": vehicle_age is not None,
            "flight_controller_connected": connected,
            "armed": bool(vehicle.get("armed", False)) if connected else False,
            "mode": str(vehicle.get("mode", "UNKNOWN")) if connected else "DISCONNECTED",
            "system_status": int(vehicle.get("system_status", 0)),
            "landed_state": extended_state if extended_age is not None else "UNKNOWN",
            "body_velocity_mps": {"x": vx, "y": vy, "z": vz},
            "body_velocity_frame": velocity_frame,
            "horizontal_direction": horizontal,
            "lateral_direction": lateral,
            "vertical_direction": vertical,
            "velocity_deadband_mps": self.velocity_deadband_mps,
            "telemetry": telemetry,
            "vehicle_state_age_s": vehicle_age,
            "velocity_age_s": velocity_age,
            "extended_state_age_s": extended_age,
            "guided_executor_age_s": executor_age,
            "executor_version": executor.get("executor_version", "legacy"),
            "action": executor.get("action"),
            "action_state": executor.get("state"),
            "action_detail": executor.get("detail"),
            **{key: value for key, value in executor.items()
               if key.startswith(("target_echo_", "tone_", "orphaned_guided_"))},
            "follow_active": bool(executor.get("follow_active", False)),
            "landing_active": bool(executor.get("landing_active", False)),
            "landing_requested": bool(executor.get("landing_requested", False)),
            "control_owner": str(executor.get("control_owner", "UNKNOWN")),
            "mode_gate": str(executor.get("mode_gate", "UNKNOWN")),
            "latest_sent_velocity_mps": executor.get("latest_sent_velocity_mps"),
            "motion_command": {
                "state": command_state,
                "source": "/mavros/setpoint_raw/local",
                "age_s": command_age,
                "body_frame": "FLU",
                "body_velocity_mps": command_body,
                "yaw_rate_rad_s": yaw_rate,
                "coordinate_frame": None if command is None else command["coordinate_frame"],
                "display_mapping": "image_up=body_forward,image_right=body_right",
            },
        }


class FlightStatusHandler(BaseHTTPRequestHandler):
    state: FlightStatusState

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path not in ("/api/status", "/api/history"):
            self.send_error(404)
            return
        content = self.state.history_snapshot() if path == "/api/history" else self.state.snapshot()
        payload = json.dumps(content, separators=(",", ":"),
                             allow_nan=path != "/api/history").encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, *_args) -> None:
        return


class FlightStatusHttp(Node):
    def __init__(self) -> None:
        super().__init__("flight_status_http")
        self.declare_parameter("http_host", "0.0.0.0")
        self.declare_parameter("http_port", 8766)
        self.declare_parameter("velocity_deadband_mps", 0.05)
        self.declare_parameter("mavros_state_topic", "/mavros/state")
        self.declare_parameter("mavros_extended_state_topic", "/mavros/extended_state")
        self.declare_parameter("mavros_body_velocity_topic", "/mavros/local_position/velocity_body")
        self.declare_parameter("guided_status_topic", "/landing/guided_executor/status")
        deadband = float(self.get_parameter("velocity_deadband_mps").value)
        if not 0.0 <= deadband <= 1.0:
            raise ValueError("velocity_deadband_mps must be in [0, 1]")
        self.state = FlightStatusState(deadband)
        FlightStatusHandler.state = self.state
        self.server = ThreadingHTTPServer(
            (
                str(self.get_parameter("http_host").value),
                int(self.get_parameter("http_port").value),
            ),
            FlightStatusHandler,
        )
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self.create_subscription(State, str(self.get_parameter("mavros_state_topic").value), self._vehicle, 10)
        extended_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(ExtendedState, str(self.get_parameter("mavros_extended_state_topic").value), self._extended, extended_qos)
        self.create_subscription(TwistStamped, str(self.get_parameter("mavros_body_velocity_topic").value), self._velocity, qos_profile_sensor_data)
        self.create_subscription(String, str(self.get_parameter("guided_status_topic").value), self._executor, 10)
        # Observe actual outgoing setpoints, not candidates or measured velocity.
        self.create_subscription(PositionTarget, "/mavros/setpoint_raw/local", self._command, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._pose, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local", self._local_velocity, qos_profile_sensor_data)
        self.create_subscription(Range, "/landing/sensor_range", self._rangefinder, qos_profile_sensor_data)
        self.create_subscription(RCIn, "/mavros/rc/in", self._rc, qos_profile_sensor_data)
        self.create_subscription(VfrHud, "/mavros/vfr_hud", self._vfr, qos_profile_sensor_data)
        self.create_timer(1.0, self.state.record_history)

    def _command(self, message: PositionTarget) -> None:
        with self.state.lock:
            self.state.command = {
                "coordinate_frame": int(message.coordinate_frame),
                "type_mask": int(message.type_mask),
                "velocity": {"x": float(message.velocity.x), "y": float(message.velocity.y), "z": float(message.velocity.z)},
                "yaw_rate": float(message.yaw_rate),
            }
            self.state.command_received_s = time.monotonic()

    def _pose(self, message: PoseStamped) -> None:
        q = message.pose.orientation
        with self.state.lock:
            self.state.pose_quaternion = (float(q.x), float(q.y), float(q.z), float(q.w))
            z = float(message.pose.position.z)
            self.state.pose_altitude_m = z if math.isfinite(z) else None
            self.state.pose_received_s = time.monotonic()

    def _local_velocity(self, message: TwistStamped) -> None:
        z = float(message.twist.linear.z)
        with self.state.lock:
            self.state.local_vertical_speed_mps = z if math.isfinite(z) else None
            self.state.local_velocity_received_s = time.monotonic()

    def _rangefinder(self, message: Range) -> None:
        value = float(message.range)
        lower, upper = float(message.min_range), float(message.max_range)
        valid = math.isfinite(value) and math.isfinite(lower) and math.isfinite(upper) and lower <= value <= upper
        with self.state.lock:
            self.state.rangefinder_m = value if valid else None
            self.state.rangefinder_received_s = time.monotonic()

    def _rc(self, message: RCIn) -> None:
        with self.state.lock:
            self.state.rc3_pwm = int(message.channels[2]) if len(message.channels) >= 3 else None
            self.state.rc_received_s = time.monotonic()

    def _vfr(self, message: VfrHud) -> None:
        throttle, altitude = float(message.throttle), float(message.altitude)
        with self.state.lock:
            self.state.motor_throttle_pct = 100.0 * throttle if math.isfinite(throttle) and 0.0 <= throttle <= 1.0 else None
            self.state.vfr_altitude_m = altitude if math.isfinite(altitude) else None
            self.state.vfr_received_s = time.monotonic()

    def _vehicle(self, message: State) -> None:
        with self.state.lock:
            self.state.vehicle = {
                "connected": bool(message.connected),
                "armed": bool(message.armed),
                "mode": str(message.mode or "UNKNOWN").upper(),
                "system_status": int(message.system_status),
            }
            self.state.vehicle_received_s = time.monotonic()

    def _extended(self, message: ExtendedState) -> None:
        with self.state.lock:
            self.state.extended_state = LANDED_STATES.get(int(message.landed_state), f"STATE_{int(message.landed_state)}")
            self.state.extended_received_s = time.monotonic()

    def _velocity(self, message: TwistStamped) -> None:
        with self.state.lock:
            self.state.velocity = {
                "x": float(message.twist.linear.x),
                "y": float(message.twist.linear.y),
                "z": float(message.twist.linear.z),
            }
            self.state.velocity_frame = str(message.header.frame_id)
            self.state.velocity_received_s = time.monotonic()

    def _executor(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        payload = normalize_executor_status(payload)
        with self.state.lock:
            self.state.executor_status = payload
            self.state.executor_received_s = time.monotonic()

    def destroy_node(self):
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlightStatusHttp()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
