"""Minimal MAVLink bridge used by both SITL UDP and Pixhawk serial profiles."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster

from .protocol import VELOCITY_YAWRATE_TYPE_MASK, clamp, ros_flu_to_body_ned
from .runtime_timing import create_steady_timer

try:
    from pymavlink import mavutil
except ImportError:  # pragma: no cover - reported clearly when the node starts
    mavutil = None


def is_flight_controller_heartbeat(message) -> bool:
    """Reject heartbeats emitted by Mission Planner or companion computers."""
    if mavutil is None:
        return False
    return (
        int(message.autopilot) != mavutil.mavlink.MAV_AUTOPILOT_INVALID
        and int(message.type) != mavutil.mavlink.MAV_TYPE_GCS
    )


def mavlink_enum_name(enum_name: str, value: int) -> str:
    """Return a readable MAVLink enum label without depending on dialect internals."""
    if mavutil is None:
        return str(value)
    try:
        return str(mavutil.mavlink.enums[enum_name][int(value)].name)
    except (AttributeError, KeyError, TypeError, ValueError):
        return str(value)


def sys_status_prearm_passed(message) -> bool:
    """Return ArduPilot's own pre-arm verdict from MAVLink SYS_STATUS.

    The companion must not infer this from GPS alone: gyro, accelerometer,
    barometer and estimator checks can still be settling after a valid fix.
    """
    if mavutil is None:
        return False
    prearm_bit = int(
        getattr(mavutil.mavlink, "MAV_SYS_STATUS_PREARM_CHECK", 1 << 28)
    )
    enabled = int(getattr(message, "onboard_control_sensors_enabled", 0))
    healthy = int(getattr(message, "onboard_control_sensors_health", 0))
    return bool(enabled & prearm_bit and healthy & prearm_bit)


def mavlink_landed_state(value: int):
    """Convert MAV_LANDED_STATE to a fail-closed optional ground flag."""
    state = int(value)
    undefined = int(getattr(mavutil.mavlink, "MAV_LANDED_STATE_UNDEFINED", 0))
    on_ground = int(getattr(mavutil.mavlink, "MAV_LANDED_STATE_ON_GROUND", 1))
    if state == undefined:
        return None
    return state == on_ground


def lifecycle_operation_enabled(
    operation: str,
    *,
    allow_lifecycle_commands: bool,
    allow_mode_commands: bool,
    allow_land_command: bool,
) -> bool:
    """Apply least privilege to flight-mode and vehicle lifecycle requests."""

    normalized = str(operation).strip().lower()
    if normalized in {"guided", "loiter", "mode"}:
        return bool(allow_mode_commands)
    if normalized == "land":
        return bool(allow_lifecycle_commands or allow_land_command)
    return bool(allow_lifecycle_commands)


def velocity_forwarding_enable_allowed(
    *, emergency_stop: bool, connected: bool, armed: bool, landed, mode: str
) -> bool:
    """Authorize external velocity streaming only for an airborne GUIDED vehicle.

    A SET_POSITION_TARGET_LOCAL_NED packet changes ArduPilot's GUIDED submode.
    Streaming even a zero target while a NAV_TAKEOFF transaction is active can
    therefore cancel motor spool-up.  This guard makes the two control domains
    mutually exclusive and also prevents ground setpoint injection.
    """

    return bool(
        not emergency_stop
        and connected
        and armed
        and landed is False
        and str(mode).upper() == "GUIDED"
    )


def flight_telemetry_ready(
    telemetry: dict,
    *,
    connected: bool,
    streams_configured: bool,
    parameters_verified: bool = True,
) -> bool:
    """Require acknowledged telemetry and flight-controller configuration."""

    return bool(
        connected
        and streams_configured
        and parameters_verified
        and telemetry.get("prearm_checks_passed", False)
        and int(telemetry.get("fix_type", 0)) >= 3
        and telemetry.get("local_position_enu_m") is not None
        and telemetry.get("relative_alt_m") is not None
    )


def parse_required_parameters(raw: str) -> dict[str, float]:
    """Parse a bounded MAVLink parameter attestation policy.

    ArduPilot parameter identifiers are at most 16 ASCII characters. Rejecting
    malformed policies at process start is safer than silently running without
    an intended flight-controller guard.
    """

    try:
        document = json.loads(str(raw))
    except (TypeError, ValueError) as error:
        raise ValueError("required_parameters_json must be a JSON object") from error
    if not isinstance(document, dict):
        raise ValueError("required_parameters_json must be a JSON object")

    required: dict[str, float] = {}
    for raw_name, raw_value in document.items():
        name = str(raw_name).strip()
        valid_name = (
            1 <= len(name) <= 16
            and name.isascii()
            and all(character == "_" or character.isdigit() or character.isupper() for character in name)
        )
        if not valid_name:
            raise ValueError(
                f"Invalid MAVLink parameter identifier {raw_name!r}; expected 1-16 uppercase ASCII characters"
            )
        if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float)):
            raise ValueError(f"Expected numeric value for MAVLink parameter {name}")
        value = float(raw_value)
        if not math.isfinite(value):
            raise ValueError(f"Expected finite value for MAVLink parameter {name}")
        required[name] = value
    return dict(sorted(required.items()))


def parameter_attestation(
    required: dict[str, float],
    observed: dict[str, float],
    *,
    tolerance: float,
) -> tuple[bool, dict[str, dict]]:
    """Compare requested flight parameters and return a serializable report."""

    absolute_tolerance = max(0.0, float(tolerance))
    report: dict[str, dict] = {}
    for name, expected in required.items():
        raw_actual = observed.get(name)
        actual = None
        if isinstance(raw_actual, (int, float)) and not isinstance(raw_actual, bool):
            candidate = float(raw_actual)
            if math.isfinite(candidate):
                actual = candidate
        matched = actual is not None and math.isclose(
            actual,
            float(expected),
            rel_tol=0.0,
            abs_tol=absolute_tolerance,
        )
        report[name] = {
            "expected": float(expected),
            "actual": actual,
            "matched": matched,
        }
    return all(item["matched"] for item in report.values()), report


def required_telemetry_intervals() -> tuple[tuple[int, int], ...]:
    """Single source of truth for the acknowledged MAVLink stream contract."""

    if mavutil is None:
        return ()
    return (
        (mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 200000),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100000),
        (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50000),
        (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50000),
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 500000),
        (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1000000),
        (mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 200000),
    )


class UavMavlinkBridge(Node):
    """Receive telemetry and optionally forward bounded body-velocity commands."""

    def __init__(self) -> None:
        super().__init__("uav_mavlink_bridge")
        self.declare_parameter("connection", "udpin:0.0.0.0:14551")
        self.declare_parameter("baud", 57600)
        self.declare_parameter("command_topic", "/uav/cmd_vel")
        self.declare_parameter("odom_topic", "/uav/odom")
        self.declare_parameter("odom_frame_id", "uav_odom")
        self.declare_parameter("base_frame_id", "uav_base_link")
        self.declare_parameter("command_enabled", False)
        self.declare_parameter("max_xy_mps", 1.0)
        self.declare_parameter("max_z_mps", 0.5)
        self.declare_parameter("max_yaw_rate_rps", 0.8)
        self.declare_parameter("command_timeout_s", 0.4)
        self.declare_parameter("heartbeat_timeout_s", 6.0)
        self.declare_parameter("telemetry_stream_max_attempts", 3)
        self.declare_parameter("required_parameters_json", "{}")
        self.declare_parameter("required_parameter_tolerance", 0.001)
        self.declare_parameter("parameter_attestation_retry_s", 1.0)
        self.declare_parameter("parameter_attestation_period_s", 10.0)
        self.declare_parameter("allow_lifecycle_commands", False)
        self.declare_parameter("allow_mode_commands", False)
        self.declare_parameter("allow_land_command", False)
        self.declare_parameter("takeoff_altitude_m", 2.8)
        self.declare_parameter("emergency_stop_topic", "/system/emergency_stop")

        if mavutil is None:
            raise RuntimeError("pymavlink is missing; install it with: pip3 install pymavlink")

        self.connection_string = str(self.get_parameter("connection").value)
        self.baud = int(self.get_parameter("baud").value)
        command_topic = str(self.get_parameter("command_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.command_enabled = bool(self.get_parameter("command_enabled").value)
        self.max_xy = float(self.get_parameter("max_xy_mps").value)
        self.max_z = float(self.get_parameter("max_z_mps").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate_rps").value)
        self.command_timeout = float(self.get_parameter("command_timeout_s").value)
        self.heartbeat_timeout = float(self.get_parameter("heartbeat_timeout_s").value)
        self.telemetry_stream_max_attempts = max(
            1, int(self.get_parameter("telemetry_stream_max_attempts").value)
        )
        self.required_parameters = parse_required_parameters(
            str(self.get_parameter("required_parameters_json").value)
        )
        self.required_parameter_tolerance = max(
            0.0, float(self.get_parameter("required_parameter_tolerance").value)
        )
        self.parameter_attestation_retry = max(
            0.5, float(self.get_parameter("parameter_attestation_retry_s").value)
        )
        self.parameter_attestation_period = max(
            self.parameter_attestation_retry,
            float(self.get_parameter("parameter_attestation_period_s").value),
        )
        self.allow_lifecycle_commands = bool(
            self.get_parameter("allow_lifecycle_commands").value
        )
        self.allow_mode_commands = bool(
            self.get_parameter("allow_mode_commands").value
        )
        self.allow_land_command = bool(
            self.get_parameter("allow_land_command").value
        )
        self.emergency_stop = False
        self.takeoff_altitude = float(
            self.get_parameter("takeoff_altitude_m").value
        )

        self.link = self._open_link(self.connection_string, self.baud)
        self.connected = False
        self.flight_controller_system_id = None
        self.flight_controller_component_id = None
        self.last_heartbeat = 0.0
        self.telemetry_stream_configured = False
        self.telemetry_stream_acknowledged = 0
        self.telemetry_stream_ack_received = 0
        self.telemetry_stream_request_attempts = 0
        self.telemetry_intervals = required_telemetry_intervals()
        self.telemetry_stream_required = len(self.telemetry_intervals)
        self.observed_parameters: dict[str, float] = {}
        self.required_parameters_verified = not self.required_parameters
        self.parameter_attestation_report = parameter_attestation(
            self.required_parameters,
            self.observed_parameters,
            tolerance=self.required_parameter_tolerance,
        )[1]
        self.parameter_request_attempts = 0
        self.last_parameter_request = 0.0
        self.last_command = 0.0
        self.have_command = False
        self.latest_command = Twist()
        self.command_inhibit_reason = (
            "" if self.command_enabled else "profile_default_disabled"
        )
        self.velocity_target_send_count = 0
        self.last_velocity_target_sent = 0.0
        self.last_velocity_target_sent_wall = 0.0
        self.last_velocity_target_was_zero = True
        self.attitude_yaw_ned = 0.0
        self.attitude_yaw_rate_ned = 0.0
        self.telemetry = {
            "connection": self.connection_string,
            "connected": False,
            "armed": False,
            "custom_mode": None,
            "fix_type": 0,
            "satellites_visible": 0,
            "relative_alt_m": None,
            "battery_v": None,
            "battery_remaining_pct": None,
            "prearm_checks_passed": False,
            "onboard_control_sensors_enabled": 0,
            "onboard_control_sensors_health": 0,
            "mode": "UNKNOWN",
            "landed": None,
            "landed_state": None,
            "landed_state_name": "MAV_LANDED_STATE_UNDEFINED",
            "rc_channels": {},
            "flight_controller_system_id": None,
            "flight_controller_component_id": None,
            "local_position_enu_m": None,
            "last_status_text": None,
            "status_texts": [],
            "last_command_ack": None,
            "command_acks": [],
            "telemetry_streams_configured": False,
            "telemetry_stream_acknowledged": 0,
            "telemetry_stream_ack_received": 0,
            "telemetry_stream_required": self.telemetry_stream_required,
            "telemetry_stream_request_attempts": 0,
            "required_parameters_verified": self.required_parameters_verified,
            "required_parameters": self.parameter_attestation_report,
            "parameter_request_attempts": 0,
        }

        self.command_subscription = self.create_subscription(
            Twist, command_topic, self.on_velocity_command, 10
        )
        self.emergency_subscription = self.create_subscription(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            self.on_emergency_stop,
            10,
        )
        self.status_publisher = self.create_publisher(String, "/uav/mavlink/status", 10)
        self.fix_publisher = self.create_publisher(NavSatFix, "/uav/gps/fix", 10)
        self.odom_publisher = self.create_publisher(Odometry, odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.enable_service = self.create_service(SetBool, "~/enable_commands", self.on_enable)
        self.guided_service = self.create_service(SetBool, "~/guided_mode", self.on_guided_mode)
        self.arm_service = self.create_service(SetBool, "~/arm", self.on_arm)
        self.takeoff_service = self.create_service(Trigger, "~/takeoff", self.on_takeoff)
        self.land_service = self.create_service(Trigger, "~/land", self.on_land)
        self.io_timer = create_steady_timer(self, 0.02, self.poll_mavlink)
        self.command_timer = create_steady_timer(self, 0.1, self.send_velocity)
        self.heartbeat_timer = create_steady_timer(
            self, 1.0, self.send_companion_heartbeat
        )
        self.telemetry_stream_timer = create_steady_timer(
            self, 5.0, self.request_telemetry_streams
        )
        self.parameter_attestation_timer = create_steady_timer(
            self, 0.5, self.request_required_parameters
        )
        self.status_timer = create_steady_timer(self, 1.0, self.publish_status)
        self.get_logger().info(
            f"MAVLink {self.connection_string}; velocity commands enabled={self.command_enabled}"
        )

    @staticmethod
    def _open_link(connection: str, baud: int):
        if connection.startswith("serial:"):
            device = connection.split(":", 1)[1]
            return mavutil.mavlink_connection(device, baud=baud, autoreconnect=True)
        return mavutil.mavlink_connection(connection, autoreconnect=True)

    def on_velocity_command(self, message: Twist) -> None:
        safe = Twist()
        safe.linear.x = clamp(message.linear.x, -self.max_xy, self.max_xy)
        safe.linear.y = clamp(message.linear.y, -self.max_xy, self.max_xy)
        safe.linear.z = clamp(message.linear.z, -self.max_z, self.max_z)
        safe.angular.z = clamp(message.angular.z, -self.max_yaw_rate, self.max_yaw_rate)
        self.latest_command = safe
        self.last_command = time.monotonic()
        self.have_command = True

    def on_emergency_stop(self, message: Bool) -> None:
        self.emergency_stop = bool(message.data)
        if self.emergency_stop:
            self.last_command = time.monotonic()
            self._inhibit_velocity_forwarding("emergency_stop")

    def on_enable(self, request: SetBool.Request, response: SetBool.Response):
        if request.data and not velocity_forwarding_enable_allowed(
            emergency_stop=self.emergency_stop,
            connected=self.connected,
            armed=bool(self.telemetry.get("armed", False)),
            landed=self.telemetry.get("landed"),
            mode=str(self.telemetry.get("mode", "UNKNOWN")),
        ):
            response.success = False
            response.message = (
                "UAV velocity forwarding rejected: requires a connected, armed, "
                "airborne GUIDED aircraft with emergency stop clear"
            )
            return response
        if request.data:
            self.command_enabled = True
            self.command_inhibit_reason = ""
        else:
            self._inhibit_velocity_forwarding("mission_or_operator_disable")
        response.success = True
        response.message = (
            "UAV velocity forwarding enabled"
            if self.command_enabled
            else "UAV velocity forwarding disabled and zeroed"
        )
        return response

    def _inhibit_velocity_forwarding(self, reason: str) -> None:
        """Atomically stop target streaming before a flight lifecycle command."""

        was_enabled = self.command_enabled
        self.command_enabled = False
        self.command_inhibit_reason = str(reason)
        self.latest_command = Twist()
        self.have_command = True
        if was_enabled:
            # Zero first, then issue the lifecycle command.  Because this node
            # spins in one executor, no later periodic velocity target can race
            # behind TAKEOFF or LAND after command_enabled becomes false.
            self._send_twist(self.latest_command)

    def on_guided_mode(self, request: SetBool.Request, response: SetBool.Response):
        mode = "GUIDED" if request.data else "LOITER"
        if not self._lifecycle_allowed(response, mode.lower()):
            return response
        response.success = self._set_mode(mode)
        response.message = (
            f"Requested {mode}" if response.success else f"Unable to request {mode}"
        )
        return response

    def _lifecycle_allowed(self, response, operation: str) -> bool:
        if not lifecycle_operation_enabled(
            operation,
            allow_lifecycle_commands=self.allow_lifecycle_commands,
            allow_mode_commands=self.allow_mode_commands,
            allow_land_command=self.allow_land_command,
        ):
            response.success = False
            response.message = (
                f"{operation} rejected: this operation is disabled in the active profile"
            )
            return False
        if not self.connected:
            response.success = False
            response.message = f"{operation} rejected: flight controller is not connected"
            return False
        if self.emergency_stop and operation not in ("land", "disarm"):
            response.success = False
            response.message = f"{operation} rejected: emergency stop is active"
            return False
        return True

    def on_arm(self, request: SetBool.Request, response: SetBool.Response):
        operation = "arm" if request.data else "disarm"
        if not self._lifecycle_allowed(response, operation):
            return response
        if request.data and not flight_telemetry_ready(
            self.telemetry,
            connected=self.connected,
            streams_configured=self.telemetry_stream_configured,
            parameters_verified=self.required_parameters_verified,
        ):
            response.success = False
            response.message = (
                "arm rejected: acknowledged telemetry and autopilot pre-arm "
                "checks are not ready"
            )
            return response
        self._inhibit_velocity_forwarding(
            "arm_transaction" if request.data else "disarm_transaction"
        )
        self.link.mav.command_long_send(
            int(self.link.target_system or 1),
            int(self.link.target_component or 1),
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if request.data else 0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        response.success = True
        response.message = f"Requested {operation}; telemetry confirmation is required"
        return response

    def on_takeoff(self, _request: Trigger.Request, response: Trigger.Response):
        if not self._lifecycle_allowed(response, "takeoff"):
            return response
        if not bool(self.telemetry.get("armed", False)):
            response.success = False
            response.message = "takeoff rejected: aircraft is not armed"
            return response
        self._inhibit_velocity_forwarding("takeoff_transaction")
        if not self._set_mode("GUIDED"):
            response.success = False
            response.message = "takeoff rejected: GUIDED mode is unavailable"
            return response
        self.link.mav.command_long_send(
            int(self.link.target_system or 1),
            int(self.link.target_component or 1),
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            self.takeoff_altitude,
        )
        response.success = True
        response.message = (
            f"Requested autopilot takeoff to {self.takeoff_altitude:.2f} m; "
            "telemetry confirmation is required"
        )
        return response

    def on_land(self, _request: Trigger.Request, response: Trigger.Response):
        if not self._lifecycle_allowed(response, "land"):
            return response
        self._inhibit_velocity_forwarding("land_transaction")
        response.success = self._set_mode("LAND")
        response.message = (
            "Requested LAND; telemetry confirmation is required"
            if response.success
            else "Unable to request LAND"
        )
        return response

    def _set_mode(self, mode: str) -> bool:
        if not self.connected:
            return False
        if str(self.telemetry.get("mode", "")).upper() == mode.upper():
            return True
        mapping = self.link.mode_mapping() or {}
        mode_id = mapping.get(mode)
        if mode_id is None:
            self.get_logger().error(f"Flight mode {mode} is not in MAVLink mode mapping")
            return False
        self.link.mav.set_mode_send(
            int(self.link.target_system or 1),
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            int(mode_id),
        )
        return True

    def poll_mavlink(self) -> None:
        for _ in range(100):
            message = self.link.recv_match(blocking=False)
            if message is None:
                break
            message_type = message.get_type()
            if message_type == "BAD_DATA":
                continue
            if message_type == "HEARTBEAT":
                if not is_flight_controller_heartbeat(message):
                    continue
                source_system = int(message.get_srcSystem())
                source_component = int(message.get_srcComponent())
                if (
                    self.flight_controller_system_id is not None
                    and source_system != self.flight_controller_system_id
                ):
                    continue
                self.flight_controller_system_id = source_system
                self.flight_controller_component_id = source_component
                self.link.target_system = source_system
                self.link.target_component = source_component
                if not self.connected:
                    self.telemetry_stream_configured = False
                    self.telemetry_stream_acknowledged = 0
                    self.telemetry_stream_ack_received = 0
                    self.telemetry_stream_request_attempts = 0
                    self.observed_parameters.clear()
                    self.required_parameters_verified = not self.required_parameters
                    self.parameter_attestation_report = parameter_attestation(
                        self.required_parameters,
                        self.observed_parameters,
                        tolerance=self.required_parameter_tolerance,
                    )[1]
                    self.parameter_request_attempts = 0
                    self.last_parameter_request = 0.0
                self.connected = True
                self.last_heartbeat = time.monotonic()
                self.telemetry["flight_controller_system_id"] = source_system
                self.telemetry["flight_controller_component_id"] = source_component
                self.telemetry["armed"] = bool(
                    message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                self.telemetry["custom_mode"] = int(message.custom_mode)
                self.telemetry["mode"] = mavutil.mode_string_v10(message)
            elif (
                self.flight_controller_system_id is not None
                and int(message.get_srcSystem()) != self.flight_controller_system_id
            ):
                continue
            elif message_type == "GPS_RAW_INT":
                self.telemetry["fix_type"] = int(message.fix_type)
                self.telemetry["satellites_visible"] = int(message.satellites_visible)
            elif message_type == "GLOBAL_POSITION_INT":
                self.telemetry["relative_alt_m"] = round(message.relative_alt / 1000.0, 3)
                self._publish_fix(message)
            elif message_type == "ATTITUDE":
                if math.isfinite(float(message.yaw)):
                    self.attitude_yaw_ned = float(message.yaw)
                if math.isfinite(float(message.yawspeed)):
                    self.attitude_yaw_rate_ned = float(message.yawspeed)
            elif message_type == "LOCAL_POSITION_NED":
                self._publish_local_odometry(message)
            elif message_type == "EXTENDED_SYS_STATE":
                landed_state = int(message.landed_state)
                self.telemetry["landed_state"] = landed_state
                self.telemetry["landed_state_name"] = mavlink_enum_name(
                    "MAV_LANDED_STATE", landed_state
                )
                self.telemetry["landed"] = mavlink_landed_state(landed_state)
            elif message_type == "SYS_STATUS":
                self.telemetry["onboard_control_sensors_enabled"] = int(
                    message.onboard_control_sensors_enabled
                )
                self.telemetry["onboard_control_sensors_health"] = int(
                    message.onboard_control_sensors_health
                )
                self.telemetry["prearm_checks_passed"] = sys_status_prearm_passed(
                    message
                )
                if message.voltage_battery != 65535:
                    self.telemetry["battery_v"] = round(message.voltage_battery / 1000.0, 2)
                if message.battery_remaining != -1:
                    self.telemetry["battery_remaining_pct"] = int(message.battery_remaining)
            elif message_type == "RC_CHANNELS":
                channels = {}
                for index in range(1, 19):
                    value = int(getattr(message, f"chan{index}_raw", 65535))
                    if value != 65535:
                        channels[str(index)] = value
                self.telemetry["rc_channels"] = channels
            elif message_type == "PARAM_VALUE":
                raw_name = message.param_id
                if isinstance(raw_name, bytes):
                    raw_name = raw_name.decode("ascii", errors="ignore")
                name = str(raw_name).rstrip("\x00")
                if name in self.required_parameters:
                    self.observed_parameters[name] = float(message.param_value)
                    (
                        self.required_parameters_verified,
                        self.parameter_attestation_report,
                    ) = parameter_attestation(
                        self.required_parameters,
                        self.observed_parameters,
                        tolerance=self.required_parameter_tolerance,
                    )
                    self.telemetry["required_parameters_verified"] = (
                        self.required_parameters_verified
                    )
                    self.telemetry["required_parameters"] = self.parameter_attestation_report
            elif message_type == "STATUSTEXT":
                status_text = message.text
                if isinstance(status_text, bytes):
                    status_text = status_text.decode("utf-8", errors="replace")
                entry = {
                    "text": str(status_text).rstrip("\x00"),
                    "severity": int(message.severity),
                    "severity_name": mavlink_enum_name(
                        "MAV_SEVERITY", int(message.severity)
                    ),
                    "received_at_s": round(time.time(), 3),
                }
                self.telemetry["last_status_text"] = entry
                history = list(self.telemetry.get("status_texts", []))
                if not history or (
                    history[-1]["text"] != entry["text"]
                    or history[-1]["severity"] != entry["severity"]
                ):
                    history.append(entry)
                    self.telemetry["status_texts"] = history[-12:]
                    self.get_logger().info(
                        f"ArduPilot {entry['severity_name']}: {entry['text']}"
                    )
            elif message_type == "COMMAND_ACK":
                command = int(message.command)
                result = int(message.result)
                entry = {
                    "command": command,
                    "command_name": mavlink_enum_name("MAV_CMD", command),
                    "result": result,
                    "result_name": mavlink_enum_name("MAV_RESULT", result),
                    "progress": int(getattr(message, "progress", 255)),
                    "result_param2": int(getattr(message, "result_param2", 0)),
                    "received_at_s": round(time.time(), 3),
                }
                if command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                    self.telemetry["stream_interval_ack_count"] = int(
                        self.telemetry.get("stream_interval_ack_count", 0)
                    ) + 1
                    self.telemetry_stream_ack_received = min(
                        self.telemetry_stream_required,
                        self.telemetry_stream_ack_received + 1,
                    )
                    if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        self.telemetry_stream_acknowledged = min(
                            self.telemetry_stream_required,
                            self.telemetry_stream_acknowledged + 1,
                        )
                    if (
                        self.telemetry_stream_ack_received
                        >= self.telemetry_stream_required
                    ):
                        self.telemetry_stream_configured = (
                            self.telemetry_stream_acknowledged
                            >= self.telemetry_stream_required
                        )
                    self.telemetry["telemetry_streams_configured"] = (
                        self.telemetry_stream_configured
                    )
                    self.telemetry["telemetry_stream_acknowledged"] = (
                        self.telemetry_stream_acknowledged
                    )
                    self.telemetry["telemetry_stream_ack_received"] = (
                        self.telemetry_stream_ack_received
                    )
                else:
                    self.telemetry["last_command_ack"] = entry
                    history = list(self.telemetry.get("command_acks", []))
                    history.append(entry)
                    self.telemetry["command_acks"] = history[-20:]

        if self.connected and time.monotonic() - self.last_heartbeat > self.heartbeat_timeout:
            self.connected = False
            self.telemetry_stream_configured = False
            self.telemetry_stream_acknowledged = 0
            self.telemetry_stream_ack_received = 0
            self.telemetry_stream_request_attempts = 0
            self.observed_parameters.clear()
            self.required_parameters_verified = not self.required_parameters
            self.parameter_attestation_report = parameter_attestation(
                self.required_parameters,
                self.observed_parameters,
                tolerance=self.required_parameter_tolerance,
            )[1]
            self.parameter_request_attempts = 0
            self.last_parameter_request = 0.0

    def _publish_fix(self, message) -> None:
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = "map"
        fix.status.status = (
            NavSatStatus.STATUS_FIX
            if int(self.telemetry["fix_type"]) >= 3
            else NavSatStatus.STATUS_NO_FIX
        )
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = message.lat / 1.0e7
        fix.longitude = message.lon / 1.0e7
        fix.altitude = message.alt / 1000.0
        self.fix_publisher.publish(fix)

    def _publish_local_odometry(self, message) -> None:
        """Publish ArduPilot LOCAL_POSITION_NED as ROS ENU odometry."""
        east = float(message.y)
        north = float(message.x)
        up = -float(message.z)
        east_velocity = float(message.vy)
        north_velocity = float(message.vx)
        up_velocity = -float(message.vz)
        yaw_enu = math.pi / 2.0 - self.attitude_yaw_ned
        cosine = math.cos(yaw_enu)
        sine = math.sin(yaw_enu)
        body_forward_velocity = cosine * east_velocity + sine * north_velocity
        body_left_velocity = -sine * east_velocity + cosine * north_velocity

        odom = Odometry()
        stamp = self.get_clock().now().to_msg()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = east
        odom.pose.pose.position.y = north
        odom.pose.pose.position.z = up
        odom.pose.pose.orientation.z = math.sin(yaw_enu / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw_enu / 2.0)
        odom.twist.twist.linear.x = body_forward_velocity
        odom.twist.twist.linear.y = body_left_velocity
        odom.twist.twist.linear.z = up_velocity
        odom.twist.twist.angular.z = -self.attitude_yaw_rate_ned
        odom.pose.covariance[0] = 0.25
        odom.pose.covariance[7] = 0.25
        odom.pose.covariance[14] = 0.36
        odom.pose.covariance[35] = 0.04
        odom.twist.covariance[0] = 0.09
        odom.twist.covariance[7] = 0.09
        odom.twist.covariance[14] = 0.16
        self.odom_publisher.publish(odom)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        transform.transform.translation.x = east
        transform.transform.translation.y = north
        transform.transform.translation.z = up
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)
        self.telemetry["local_position_enu_m"] = [
            round(east, 3),
            round(north, 3),
            round(up, 3),
        ]

    def send_velocity(self) -> None:
        if self.emergency_stop:
            self._send_twist(Twist())
            return
        if not self.command_enabled or not self.connected or not self.have_command:
            return
        fresh = (time.monotonic() - self.last_command) <= self.command_timeout
        self._send_twist(self.latest_command if fresh else Twist())

    def send_companion_heartbeat(self) -> None:
        """Keep UDP sessions alive and identify this node as the companion computer."""
        self.link.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def request_telemetry_streams(self) -> None:
        """Request the safety-critical telemetry used by the tracker."""
        if (
            not self.connected
            or self.telemetry_stream_configured
            or self.telemetry_stream_request_attempts
            >= self.telemetry_stream_max_attempts
        ):
            return
        self.telemetry_stream_request_attempts += 1
        self.telemetry_stream_acknowledged = 0
        self.telemetry_stream_ack_received = 0
        self.telemetry["telemetry_stream_request_attempts"] = (
            self.telemetry_stream_request_attempts
        )
        self.telemetry["telemetry_stream_acknowledged"] = 0
        self.telemetry["telemetry_stream_ack_received"] = 0
        for message_id, interval_us in self.telemetry_intervals:
            self.link.mav.command_long_send(
                int(self.link.target_system or 1),
                int(self.link.target_component or 1),
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                interval_us,
                0,
                0,
                0,
                0,
                0,
            )

    def request_required_parameters(self) -> None:
        """Continuously attest safety-critical autopilot parameters.

        Values are never changed from the companion. A missing or mismatched
        value keeps preflight closed; the operator must correct the flight
        controller configuration through the commissioned maintenance path.
        """

        if not self.connected or not self.required_parameters:
            return
        now = time.monotonic()
        request_interval = (
            self.parameter_attestation_period
            if self.required_parameters_verified
            else self.parameter_attestation_retry
        )
        if now - self.last_parameter_request < request_interval:
            return
        self.last_parameter_request = now
        self.parameter_request_attempts += 1
        target_system = int(self.flight_controller_system_id or self.link.target_system or 1)
        target_component = int(
            self.flight_controller_component_id or self.link.target_component or 1
        )
        for name in self.required_parameters:
            self.link.mav.param_request_read_send(
                target_system,
                target_component,
                name.encode("ascii"),
                -1,
            )
        self.telemetry["parameter_request_attempts"] = self.parameter_request_attempts

    def _send_twist(self, command: Twist) -> None:
        if not self.connected:
            return
        converted = ros_flu_to_body_ned(
            command.linear.x,
            command.linear.y,
            command.linear.z,
            command.angular.z,
        )
        self.link.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            int(self.link.target_system or 1),
            int(self.link.target_component or 1),
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            VELOCITY_YAWRATE_TYPE_MASK,
            0.0,
            0.0,
            0.0,
            converted.forward,
            converted.right,
            converted.down,
            0.0,
            0.0,
            0.0,
            0.0,
            converted.yaw_rate_clockwise,
        )
        self.velocity_target_send_count += 1
        self.last_velocity_target_sent = time.monotonic()
        self.last_velocity_target_sent_wall = time.time()
        self.last_velocity_target_was_zero = all(
            abs(value) <= 1.0e-9
            for value in (
                converted.forward,
                converted.right,
                converted.down,
                converted.yaw_rate_clockwise,
            )
        )

    def publish_status(self) -> None:
        self.telemetry["connected"] = self.connected
        self.telemetry["telemetry_streams_configured"] = (
            self.telemetry_stream_configured
        )
        self.telemetry["telemetry_stream_configuration_failed"] = bool(
            not self.telemetry_stream_configured
            and self.telemetry_stream_request_attempts
            >= self.telemetry_stream_max_attempts
        )
        self.telemetry["telemetry_stream_acknowledged"] = (
            self.telemetry_stream_acknowledged
        )
        self.telemetry["telemetry_stream_ack_received"] = (
            self.telemetry_stream_ack_received
        )
        self.telemetry["telemetry_stream_request_attempts"] = (
            self.telemetry_stream_request_attempts
        )
        self.telemetry["flight_ready"] = flight_telemetry_ready(
            self.telemetry,
            connected=self.connected,
            streams_configured=self.telemetry_stream_configured,
            parameters_verified=self.required_parameters_verified,
        )
        self.telemetry["required_parameters_verified"] = (
            self.required_parameters_verified
        )
        self.telemetry["required_parameters"] = self.parameter_attestation_report
        self.telemetry["parameter_request_attempts"] = self.parameter_request_attempts
        self.telemetry["command_enabled"] = self.command_enabled
        self.telemetry["command_inhibit_reason"] = self.command_inhibit_reason
        self.telemetry["velocity_target_send_count"] = self.velocity_target_send_count
        self.telemetry["last_velocity_target_sent_at_s"] = (
            None
            if self.last_velocity_target_sent_wall == 0.0
            else round(self.last_velocity_target_sent_wall, 3)
        )
        self.telemetry["last_velocity_target_age_s"] = (
            None
            if self.last_velocity_target_sent == 0.0
            else round(max(0.0, time.monotonic() - self.last_velocity_target_sent), 3)
        )
        self.telemetry["last_velocity_target_was_zero"] = (
            self.last_velocity_target_was_zero
        )
        self.telemetry["emergency_stop"] = self.emergency_stop
        self.telemetry["lifecycle_commands_allowed"] = self.allow_lifecycle_commands
        self.telemetry["mode_commands_allowed"] = self.allow_mode_commands
        self.telemetry["land_command_allowed"] = self.allow_land_command
        self.telemetry["last_heartbeat_age_s"] = (
            None
            if self.last_heartbeat == 0.0
            else round(time.monotonic() - self.last_heartbeat, 2)
        )
        message = String()
        message.data = json.dumps(self.telemetry, ensure_ascii=False, sort_keys=True)
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UavMavlinkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.command_enabled = False
        node._send_twist(Twist())
        node.link.close()
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
