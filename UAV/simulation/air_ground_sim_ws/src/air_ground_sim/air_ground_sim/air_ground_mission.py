"""ROS 2 orchestration for the complete UAV-UGV cooperative mission."""

import json
import math
import time
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Empty, Float64, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .mission_logic import (
    acknowledged_retry_deadline,
    distance_speed_scale,
    dock_attach_authorized,
    mavlink_command_ack_outcome,
    mission_state_allows_ugv_motion,
    mission_start_is_safe,
    mission_terminal_reset_is_safe,
    mission_plan_is_commissioned,
    MissionFacts,
    MissionState,
    moving_deck_envelope,
    navigation_goal_failed,
    next_state,
    parse_detachable_joint_state,
    progress_watchdog_step,
    split_speed_scale,
    STATE_TIMEOUTS,
    sustained_for,
    transform_stamp_is_fresh,
    update_sustained_since,
)
from .ros_compat import run_shutdown_action
from .runtime_timing import create_steady_timer


class AirGroundMission(Node):
    """Run and expose the guarded multi-stage cooperative operation."""

    def __init__(self) -> None:
        super().__init__("air_ground_mission")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("auto_start_delay_s", 12.0)
        self.declare_parameter("simulation_lifecycle", True)
        self.declare_parameter("mission_plan_validated", False)
        self.declare_parameter("mission_plan_id", "UNCOMMISSIONED")
        self.declare_parameter("initial_ugv_x", -9.0)
        self.declare_parameter("initial_ugv_y", -6.0)
        self.declare_parameter("transit_ugv_x", 4.8)
        self.declare_parameter("transit_ugv_y", 5.2)
        self.declare_parameter("transit_ugv_yaw", 0.70)
        self.declare_parameter("transit_uav_x", 4.2)
        self.declare_parameter("transit_uav_y", 4.5)
        self.declare_parameter("transit_uav_z", 3.2)
        self.declare_parameter("moving_ugv_x", -7.0)
        self.declare_parameter("moving_ugv_y", 9.0)
        self.declare_parameter("moving_ugv_yaw", math.pi)
        self.declare_parameter("ride_ugv_x", 0.0)
        self.declare_parameter("ride_ugv_y", 9.0)
        self.declare_parameter("ride_ugv_yaw", math.pi)
        self.declare_parameter("start_dock_altitude_m", 2.6)
        self.declare_parameter("timeout_scale", 1.0)
        self.declare_parameter("state_timeout_overrides_json", "{}")
        self.declare_parameter("follow_ugv_speed_scale", 0.15)
        self.declare_parameter("docking_ugv_speed_scale", 0.15)
        self.declare_parameter("ride_final_ugv_speed_scale", 0.08)
        self.declare_parameter("ride_slowdown_distance_m", 2.0)
        self.declare_parameter("moving_dock_max_yaw_error_rad", 0.35)
        self.declare_parameter("moving_dock_max_yaw_rate_rps", 0.20)
        self.declare_parameter("moving_dock_entry_max_yaw_error_rad", 0.20)
        self.declare_parameter("moving_dock_entry_max_yaw_rate_rps", 0.12)
        self.declare_parameter("moving_dock_entry_hold_s", 3.0)
        self.declare_parameter("ugv_map_pose_timeout_s", 1.0)
        self.declare_parameter("ugv_progress_timeout_s", 20.0)
        self.declare_parameter("ugv_progress_min_distance_m", 0.15)
        self.declare_parameter("reset_stopped_speed_mps", 0.03)
        self.declare_parameter("completion_stopped_speed_mps", 0.03)
        self.declare_parameter("completion_stopped_hold_s", 2.0)
        self.declare_parameter("arm_retry_s", 3.0)
        self.declare_parameter("arm_failure_cooldown_s", 15.0)
        self.declare_parameter("arm_confirmation_timeout_s", 10.0)
        self.declare_parameter("arm_max_attempts", 3)
        self.declare_parameter("takeoff_retry_s", 5.0)
        self.declare_parameter("takeoff_failure_cooldown_s", 10.0)
        self.declare_parameter("takeoff_confirmation_timeout_s", 15.0)
        self.declare_parameter("takeoff_disarm_grace_s", 3.0)
        self.declare_parameter("takeoff_max_attempts", 3)
        self.declare_parameter("land_retry_s", 2.0)
        self.declare_parameter("land_max_attempts", 3)
        self.declare_parameter("moving_ugv_min_speed_mps", 0.04)
        self.declare_parameter("moving_ugv_confirm_s", 2.0)
        self.declare_parameter("moving_capture_max_altitude_m", 0.50)
        self.declare_parameter("require_system_ready", True)
        self.declare_parameter("system_health_timeout_s", 1.0)
        self.declare_parameter("system_ready_hold_s", 2.0)
        self.declare_parameter("preflight_ready_hold_s", 2.0)
        self.declare_parameter("mission_gate_topic", "/ugv/mission_gate")
        self.declare_parameter("emergency_stop_topic", "/system/emergency_stop")

        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.auto_start_delay = float(self.get_parameter("auto_start_delay_s").value)
        self.simulation_lifecycle = bool(
            self.get_parameter("simulation_lifecycle").value
        )
        self.mission_plan_validated = bool(
            self.get_parameter("mission_plan_validated").value
        )
        self.mission_plan_id = str(self.get_parameter("mission_plan_id").value)
        self.initial_ugv = (
            float(self.get_parameter("initial_ugv_x").value),
            float(self.get_parameter("initial_ugv_y").value),
        )
        self.transit_ugv = (
            float(self.get_parameter("transit_ugv_x").value),
            float(self.get_parameter("transit_ugv_y").value),
            float(self.get_parameter("transit_ugv_yaw").value),
        )
        self.transit_uav = (
            float(self.get_parameter("transit_uav_x").value),
            float(self.get_parameter("transit_uav_y").value),
            float(self.get_parameter("transit_uav_z").value),
        )
        self.moving_ugv = (
            float(self.get_parameter("moving_ugv_x").value),
            float(self.get_parameter("moving_ugv_y").value),
            float(self.get_parameter("moving_ugv_yaw").value),
        )
        self.ride_ugv = (
            float(self.get_parameter("ride_ugv_x").value),
            float(self.get_parameter("ride_ugv_y").value),
            float(self.get_parameter("ride_ugv_yaw").value),
        )
        self.start_dock_altitude = float(
            self.get_parameter("start_dock_altitude_m").value
        )
        self.timeout_scale = max(float(self.get_parameter("timeout_scale").value), 0.1)
        try:
            timeout_overrides = json.loads(
                str(self.get_parameter("state_timeout_overrides_json").value)
            )
            if not isinstance(timeout_overrides, dict):
                raise ValueError("must be a JSON object")
            self.state_timeout_overrides = {}
            for state_name, value in timeout_overrides.items():
                state = MissionState(str(state_name))
                timeout = float(value)
                if state not in STATE_TIMEOUTS or not math.isfinite(timeout) or timeout <= 0.0:
                    raise ValueError(f"invalid timeout for {state_name}")
                self.state_timeout_overrides[state] = timeout
        except (json.JSONDecodeError, TypeError, ValueError) as error:
            raise ValueError(f"state_timeout_overrides_json is invalid: {error}") from error
        self.follow_ugv_speed_scale = max(
            0.05, min(1.0, float(self.get_parameter("follow_ugv_speed_scale").value))
        )
        self.docking_ugv_speed_scale = max(
            0.05,
            min(1.0, float(self.get_parameter("docking_ugv_speed_scale").value)),
        )
        self.ride_final_ugv_speed_scale = max(
            0.05,
            min(1.0, float(self.get_parameter("ride_final_ugv_speed_scale").value)),
        )
        self.ride_slowdown_distance = max(
            0.1, float(self.get_parameter("ride_slowdown_distance_m").value)
        )
        self.moving_dock_max_yaw_error = max(
            0.0,
            float(self.get_parameter("moving_dock_max_yaw_error_rad").value),
        )
        self.moving_dock_max_yaw_rate = max(
            0.0,
            float(self.get_parameter("moving_dock_max_yaw_rate_rps").value),
        )
        self.moving_dock_entry_max_yaw_error = min(
            self.moving_dock_max_yaw_error,
            max(
                0.0,
                float(
                    self.get_parameter("moving_dock_entry_max_yaw_error_rad").value
                ),
            ),
        )
        self.moving_dock_entry_max_yaw_rate = min(
            self.moving_dock_max_yaw_rate,
            max(
                0.0,
                float(
                    self.get_parameter("moving_dock_entry_max_yaw_rate_rps").value
                ),
            ),
        )
        self.moving_dock_entry_hold = max(
            0.0, float(self.get_parameter("moving_dock_entry_hold_s").value)
        )
        self.ugv_map_pose_timeout = max(
            0.1, float(self.get_parameter("ugv_map_pose_timeout_s").value)
        )
        self.ugv_progress_timeout = max(
            1.0, float(self.get_parameter("ugv_progress_timeout_s").value)
        )
        self.ugv_progress_min_distance = max(
            0.01, float(self.get_parameter("ugv_progress_min_distance_m").value)
        )
        self.reset_stopped_speed = max(
            0.0, float(self.get_parameter("reset_stopped_speed_mps").value)
        )
        self.completion_stopped_speed = max(
            0.0, float(self.get_parameter("completion_stopped_speed_mps").value)
        )
        self.completion_stopped_hold = max(
            0.0, float(self.get_parameter("completion_stopped_hold_s").value)
        )
        self.arm_retry = max(
            0.5, float(self.get_parameter("arm_retry_s").value)
        )
        self.arm_failure_cooldown = max(
            self.arm_retry,
            float(self.get_parameter("arm_failure_cooldown_s").value),
        )
        self.arm_confirmation_timeout = max(
            self.arm_retry,
            float(self.get_parameter("arm_confirmation_timeout_s").value),
        )
        self.arm_max_attempts = max(
            1, int(self.get_parameter("arm_max_attempts").value)
        )
        self.takeoff_retry = max(
            1.0, float(self.get_parameter("takeoff_retry_s").value)
        )
        self.takeoff_failure_cooldown = max(
            self.takeoff_retry,
            float(self.get_parameter("takeoff_failure_cooldown_s").value),
        )
        self.takeoff_confirmation_timeout = max(
            self.takeoff_retry,
            float(self.get_parameter("takeoff_confirmation_timeout_s").value),
        )
        self.takeoff_disarm_grace = max(
            0.5, float(self.get_parameter("takeoff_disarm_grace_s").value)
        )
        self.takeoff_max_attempts = max(
            1, int(self.get_parameter("takeoff_max_attempts").value)
        )
        self.land_retry = max(
            0.5, float(self.get_parameter("land_retry_s").value)
        )
        self.land_max_attempts = max(
            1, int(self.get_parameter("land_max_attempts").value)
        )
        self.moving_ugv_min_speed = max(
            0.01, float(self.get_parameter("moving_ugv_min_speed_mps").value)
        )
        self.moving_ugv_confirm = max(
            0.2, float(self.get_parameter("moving_ugv_confirm_s").value)
        )
        self.moving_capture_max_altitude = max(
            0.0,
            float(self.get_parameter("moving_capture_max_altitude_m").value),
        )
        self.require_system_ready = bool(
            self.get_parameter("require_system_ready").value
        )
        self.system_health_timeout = max(
            0.1, float(self.get_parameter("system_health_timeout_s").value)
        )
        self.system_ready_hold = max(
            0.0, float(self.get_parameter("system_ready_hold_s").value)
        )
        self.preflight_ready_hold = max(
            0.0, float(self.get_parameter("preflight_ready_hold_s").value)
        )

        self.state = MissionState.IDLE
        self.state_started = time.monotonic()
        self.started_at = 0.0
        self.paused = False
        self.reason = "ready"
        self.transition_count = 0
        self.last_action_drive = 0.0
        self.telemetry = {}
        self.navigation = {}
        self.docking = {}
        self.perception = {}
        self.ugv_goal_done = False
        self.ugv_goal_status = "idle"
        self.ugv_goal_handle = None
        self.ugv_goal_generation = 0
        self.dock_detached = None
        # The armed-capture safety deadline starts from positive mechanical
        # attachment feedback, not from entry into a mission state.  Those two
        # events are asynchronous on both Gazebo and physical latch hardware.
        self.dock_attached_since = 0.0
        self.last_uav_goal = None
        self.action_dispatched_for_state = None
        self.last_arm_request = 0.0
        self.last_arm_request_wall = 0.0
        self.last_arm_ack_wall = 0.0
        self.arm_retry_not_before = 0.0
        self.last_arm_ack_outcome = None
        self.arm_request_attempts = 0
        self.last_disarm_request = 0.0
        self.disarm_request_attempts = 0
        self.last_takeoff_request = 0.0
        self.last_takeoff_request_wall = 0.0
        self.last_takeoff_ack_wall = 0.0
        self.takeoff_retry_not_before = 0.0
        self.last_takeoff_ack_outcome = None
        self.takeoff_request_attempts = 0
        self.last_land_request = 0.0
        self.land_request_attempts = 0
        self.ugv_speed_scale = 0.0
        self.ugv_measured_speed = 0.0
        self.ugv_map_position = None
        self.ugv_map_pose_time = 0.0
        self.ugv_map_pose_source = "none"
        self.ugv_map_transform_age = None
        self.ugv_yaw = None
        self.ugv_yaw_rate = 0.0
        self.ugv_moving_since = 0.0
        self.ugv_dock_entry_envelope_since = 0.0
        self.ugv_progress_anchor = None
        self.ugv_progress_since = None
        self.ugv_progress_stalled = False
        self.completion_stopped_since = 0.0
        self.system_health = {}
        self.system_health_time = 0.0
        self.system_ready_since = 0.0
        self.preflight_ready_since = 0.0
        self.emergency_stop = False

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.status_publisher = self.create_publisher(String, "/mission/status", 10)
        self.event_publisher = self.create_publisher(String, "/mission/events", 10)
        self.uav_goal_publisher = self.create_publisher(
            PoseStamped, "/uav/nav/goal", 10
        )
        self.docking_mode_publisher = self.create_publisher(
            String, "/uav/docking/mode", 10
        )
        self.attach_publisher = self.create_publisher(Empty, "/uav/dock/attach", 10)
        self.detach_publisher = self.create_publisher(Empty, "/uav/dock/detach", 10)
        self.ugv_speed_scale_publisher = self.create_publisher(
            Float64, str(self.get_parameter("mission_gate_topic").value), 10
        )
        self.ugv_nav2_speed_limit_publisher = self.create_publisher(
            SpeedLimit, "/speed_limit", 10
        )

        self.create_subscription(String, "/uav/mavlink/status", self.on_telemetry, 10)
        self.create_subscription(String, "/uav/navigation/status", self.on_navigation, 10)
        self.create_subscription(String, "/uav/docking/status", self.on_docking, 10)
        self.create_subscription(String, "/uav/perception/status", self.on_perception, 10)
        self.create_subscription(String, "/uav/dock/detached", self.on_dock_state, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_ugv_odom, 20)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_ugv_map_pose, 10
        )
        self.create_subscription(String, "/system/health", self.on_system_health, 10)
        self.create_subscription(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            self.on_emergency_stop,
            10,
        )

        self.nav_enable_client = self.create_client(SetBool, "/uav_navigation/enable")
        self.docking_enable_client = self.create_client(
            SetBool, "/uav_docking_controller/enable"
        )
        self.velocity_forward_enable_client = self.create_client(
            SetBool, "/uav_mavlink_bridge/enable_commands"
        )
        self.guided_client = self.create_client(
            SetBool, "/uav_mavlink_bridge/guided_mode"
        )
        self.arm_client = self.create_client(SetBool, "/uav_mavlink_bridge/arm")
        self.takeoff_client = self.create_client(Trigger, "/uav_mavlink_bridge/takeoff")
        self.land_client = self.create_client(Trigger, "/uav_mavlink_bridge/land")
        self.gimbal_down_client = self.create_client(
            Trigger, "/uav_gimbal_controller/look_down"
        )
        self.gimbal_center_client = self.create_client(
            Trigger, "/uav_gimbal_controller/center"
        )
        self.navigate_action = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.create_service(Trigger, "~/start", self.on_start)
        self.create_service(SetBool, "~/pause", self.on_pause)
        self.create_service(Trigger, "~/abort", self.on_abort)
        self.create_service(Trigger, "~/fault", self.on_fault)
        self.create_service(Trigger, "~/reset", self.on_reset)
        self.control_timer = create_steady_timer(self, 0.2, self.control_tick)
        self.status_timer = create_steady_timer(self, 0.25, self.publish_status)
        self.gate_heartbeat_timer = create_steady_timer(
            self, 0.1, self.publish_gate_heartbeat
        )
        self.boot_time = time.monotonic()

    def _parse(self, message: String) -> dict:
        try:
            return json.loads(message.data)
        except json.JSONDecodeError:
            return {}

    def on_telemetry(self, message: String) -> None:
        self.telemetry = self._parse(message)
        now = time.monotonic()
        self.preflight_ready_since = update_sustained_since(
            bool(self.telemetry.get("flight_ready", False)),
            self.preflight_ready_since,
            now,
        )

    def on_navigation(self, message: String) -> None:
        self.navigation = self._parse(message)

    def on_docking(self, message: String) -> None:
        self.docking = self._parse(message)

    def on_perception(self, message: String) -> None:
        self.perception = self._parse(message)

    def on_system_health(self, message: String) -> None:
        self.system_health = self._parse(message)
        now = time.monotonic()
        self.system_health_time = now
        instant_ready = bool(
            self.system_health.get("ready", False)
            and not self.system_health.get("emergency_stop", False)
        )
        self.system_ready_since = update_sustained_since(
            instant_ready,
            self.system_ready_since,
            now,
        )

    def on_emergency_stop(self, message: Bool) -> None:
        self.emergency_stop = bool(message.data)
        if self.emergency_stop and self.state not in (
            MissionState.IDLE,
            MissionState.COMPLETE,
            MissionState.ABORTED,
            MissionState.FAULT,
        ):
            self._abort("system_emergency_stop", MissionState.FAULT)

    def on_dock_state(self, message: String) -> None:
        parsed = parse_detachable_joint_state(message.data)
        if parsed is not None:
            if parsed is False and self.dock_detached is not False:
                self.dock_attached_since = time.monotonic()
            elif parsed is True:
                self.dock_attached_since = 0.0
            self.dock_detached = parsed

    def on_ugv_odom(self, message: Odometry) -> None:
        velocity = message.twist.twist.linear
        self.ugv_measured_speed = math.hypot(float(velocity.x), float(velocity.y))
        self.ugv_yaw_rate = float(message.twist.twist.angular.z)
        now = time.monotonic()
        if self.ugv_measured_speed >= self.moving_ugv_min_speed:
            if self.ugv_moving_since == 0.0:
                self.ugv_moving_since = now
        else:
            self.ugv_moving_since = 0.0

    def on_ugv_map_pose(self, message: PoseWithCovarianceStamped) -> None:
        position = message.pose.pose.position
        self.ugv_map_position = (float(position.x), float(position.y))
        orientation = message.pose.pose.orientation
        sin_yaw = 2.0 * (
            float(orientation.w) * float(orientation.z)
            + float(orientation.x) * float(orientation.y)
        )
        cos_yaw = 1.0 - 2.0 * (
            float(orientation.y) ** 2 + float(orientation.z) ** 2
        )
        self.ugv_yaw = math.atan2(sin_yaw, cos_yaw)
        self.ugv_map_pose_time = time.monotonic()
        self.ugv_map_pose_source = "amcl_fallback"

    def _refresh_ugv_map_pose_from_tf(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException:
            return
        stamp = transform.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        now_ns = self.get_clock().now().nanoseconds
        self.ugv_map_transform_age = (now_ns - stamp_ns) / 1_000_000_000.0
        if not transform_stamp_is_fresh(
            now_ns=now_ns,
            stamp_ns=stamp_ns,
            timeout_s=self.ugv_map_pose_timeout,
        ):
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.ugv_map_position = (float(translation.x), float(translation.y))
        sin_yaw = 2.0 * (
            float(rotation.w) * float(rotation.z)
            + float(rotation.x) * float(rotation.y)
        )
        cos_yaw = 1.0 - 2.0 * (
            float(rotation.y) ** 2 + float(rotation.z) ** 2
        )
        self.ugv_yaw = math.atan2(sin_yaw, cos_yaw)
        self.ugv_map_pose_time = time.monotonic()
        self.ugv_map_pose_source = "tf_map_to_base_link"

    def _set_ugv_speed_scale(self, value: float) -> None:
        scale, nav2_percentage, safety_gate = split_speed_scale(value)
        self.ugv_speed_scale = scale

        limit = SpeedLimit()
        limit.header.stamp = self.get_clock().now().to_msg()
        limit.percentage = True
        limit.speed_limit = nav2_percentage

        gate = Float64()
        gate.data = safety_gate

        # Opening sequence: establish the upstream limit before commands can
        # reach hardware. Closing sequence: stop at the final gate first.
        if safety_gate > 0.0:
            self.ugv_nav2_speed_limit_publisher.publish(limit)
            self.ugv_speed_scale_publisher.publish(gate)
        else:
            self.ugv_speed_scale_publisher.publish(gate)
            self.ugv_nav2_speed_limit_publisher.publish(limit)

    def publish_gate_heartbeat(self) -> None:
        gate = Float64()
        gate.data = 1.0 if self.ugv_speed_scale > 0.0 else 0.0
        self.ugv_speed_scale_publisher.publish(gate)

    def _system_ready(self) -> bool:
        now = time.monotonic()
        if not self.require_system_ready:
            return not self.emergency_stop
        fresh = (
            self.system_health_time > 0.0
            and now - self.system_health_time <= self.system_health_timeout
        )
        instant_ready = bool(
            fresh
            and bool(self.system_health.get("ready", False))
            and not bool(self.system_health.get("emergency_stop", False))
            and not self.emergency_stop
        )
        return sustained_for(
            instant_ready,
            self.system_ready_since,
            now,
            self.system_ready_hold,
        )

    def _preflight_ready(self) -> bool:
        now = time.monotonic()
        instant_ready = bool(self.telemetry.get("flight_ready", False))
        return sustained_for(
            instant_ready,
            self.preflight_ready_since,
            now,
            self.preflight_ready_hold,
        )

    def on_start(self, _request: Trigger.Request, response: Trigger.Response):
        if self.state != MissionState.IDLE:
            response.success = False
            response.message = (
                f"Mission start rejected in {self.state.value}; perform a guarded "
                "mission reset after reaching a safe terminal condition"
            )
            return response
        if not mission_plan_is_commissioned(
            simulation_lifecycle=self.simulation_lifecycle,
            validated=self.mission_plan_validated,
            plan_id=self.mission_plan_id,
        ):
            response.success = False
            response.message = (
                "Mission start rejected: a commissioned site mission plan is required"
            )
            return response
        if not self._system_ready():
            response.success = False
            response.message = "Mission start rejected: system readiness gate is not satisfied"
            return response
        if not mission_start_is_safe(
            self.state,
            armed=bool(self.telemetry.get("armed", False)),
            landed=self.telemetry.get("landed"),
            ugv_speed_mps=self.ugv_measured_speed,
            stopped_speed_mps=self.reset_stopped_speed,
        ):
            response.success = False
            response.message = (
                "Mission start rejected: aircraft must be positively landed and "
                "disarmed, and the ground vehicle must be stopped"
            )
            return response
        self.started_at = time.monotonic()
        self.ugv_goal_done = False
        self.ugv_goal_status = "idle"
        self.paused = False
        self._set_controller(self.velocity_forward_enable_client, False)
        self._set_ugv_speed_scale(0.0)
        self._transition(MissionState.RELEASE_REMOTE_DOCK, "mission_started")
        response.success = True
        response.message = "Cooperative mission started"
        return response

    def on_pause(self, request: SetBool.Request, response: SetBool.Response):
        self.paused = bool(request.data)
        self._set_controller(self.nav_enable_client, False)
        self._set_controller(self.docking_enable_client, False)
        self._set_controller(self.velocity_forward_enable_client, False)
        if self.paused:
            self._set_ugv_speed_scale(0.0)
            self.reason = "operator_paused"
            self.cancel_ugv_goal()
        else:
            self.reason = "operator_resumed"
            self.action_dispatched_for_state = None
            self.state_started = time.monotonic()
        response.success = True
        response.message = "Mission paused" if self.paused else "Mission resumed"
        return response

    def on_abort(self, _request: Trigger.Request, response: Trigger.Response):
        self._abort("operator_abort")
        response.success = True
        response.message = "Mission aborted; motion commands and ground-vehicle gates are zeroed"
        return response

    def on_fault(self, _request: Trigger.Request, response: Trigger.Response):
        self._abort("system_supervisor_fault", MissionState.FAULT)
        response.success = True
        response.message = "Mission faulted by system supervisor; motion gates closed"
        return response

    def on_reset(self, _request: Trigger.Request, response: Trigger.Response):
        if not mission_terminal_reset_is_safe(
            self.state,
            armed=bool(self.telemetry.get("armed", False)),
            landed=self.telemetry.get("landed"),
            ugv_speed_mps=self.ugv_measured_speed,
            stopped_speed_mps=self.reset_stopped_speed,
        ):
            response.success = False
            response.message = (
                "Mission reset rejected: a terminal state, positive landed/disarmed "
                "telemetry and a stopped ground vehicle are all required"
            )
            return response
        self.cancel_ugv_goal()
        self._set_controller(self.nav_enable_client, False)
        self._set_controller(self.docking_enable_client, False)
        self._set_controller(self.velocity_forward_enable_client, False)
        self._set_ugv_speed_scale(0.0)
        self.started_at = 0.0
        self.paused = False
        self._transition(MissionState.IDLE, "reset")
        response.success = True
        response.message = "Mission reset"
        return response

    def _set_controller(self, client, enabled: bool) -> None:
        if not client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = bool(enabled)
        client.call_async(request)

    def _trigger(self, client) -> None:
        if client.service_is_ready():
            client.call_async(Trigger.Request())

    def _set_mode(self, mode: str) -> None:
        message = String()
        message.data = mode
        self.docking_mode_publisher.publish(message)

    def _publish_uav_goal(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "uav_odom"
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = float(z)
        goal.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.orientation.w = math.cos(float(yaw) / 2.0)
        self.last_uav_goal = [float(x), float(y), float(z), float(yaw)]
        self.uav_goal_publisher.publish(goal)

    def send_ugv_goal(self, x: float, y: float, yaw: float) -> None:
        if not self.navigate_action.server_is_ready():
            self.ugv_goal_status = "waiting_nav2_action"
            return
        self.ugv_goal_generation += 1
        generation = self.ugv_goal_generation
        previous_handle = self.ugv_goal_handle
        self.ugv_goal_handle = None
        if previous_handle is not None:
            previous_handle.cancel_goal_async()
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        self.ugv_goal_done = False
        self.ugv_goal_status = "sending"
        future = self.navigate_action.send_goal_async(goal)
        future.add_done_callback(
            lambda completed, goal_generation=generation: self.on_ugv_goal_response(
                completed, goal_generation
            )
        )

    def on_ugv_goal_response(self, future, generation: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            if generation != self.ugv_goal_generation:
                return
            self.ugv_goal_status = f"send_error:{error}"
            return
        if generation != self.ugv_goal_generation:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        self.ugv_goal_handle = goal_handle
        if not goal_handle.accepted:
            self.ugv_goal_status = "rejected"
            return
        self.ugv_goal_status = "executing"
        result = goal_handle.get_result_async()
        result.add_done_callback(
            lambda completed, goal_generation=generation: self.on_ugv_goal_result(
                completed, goal_generation
            )
        )

    def on_ugv_goal_result(self, future, generation: int) -> None:
        if generation != self.ugv_goal_generation:
            return
        try:
            status = int(future.result().status)
        except Exception as error:
            self.ugv_goal_status = f"result_error:{error}"
            return
        self.ugv_goal_done = status == GoalStatus.STATUS_SUCCEEDED
        self.ugv_goal_status = "succeeded" if self.ugv_goal_done else f"ended_{status}"

    def cancel_ugv_goal(self) -> None:
        self.ugv_goal_generation += 1
        if self.ugv_goal_handle is not None:
            self.ugv_goal_handle.cancel_goal_async()
        self.ugv_goal_handle = None

    def _facts(self) -> MissionFacts:
        altitude = self.telemetry.get("relative_alt_m")
        altitude = 0.0 if altitude is None else float(altitude)
        separation = self.docking.get("separation_m")
        ugv_moving = (
            self.ugv_moving_since > 0.0
            and time.monotonic() - self.ugv_moving_since >= self.moving_ugv_confirm
        )
        return MissionFacts(
            connected=bool(self.telemetry.get("connected", False)),
            flight_ready=self._preflight_ready(),
            armed=bool(self.telemetry.get("armed", False)),
            landed=self.telemetry.get("landed"),
            altitude_m=altitude,
            navigation_reached=bool(self.navigation.get("goal_reached", False)),
            docking_capture_ready=bool(self.docking.get("capture_ready", False)),
            dock_detached=self.dock_detached,
            ugv_goal_done=self.ugv_goal_done,
            ugv_stopped_stable=self._completion_stop_ready(),
            ugv_moving=ugv_moving,
            ugv_motion_envelope=(
                self._ugv_dock_entry_envelope_ready()
                if self.state == MissionState.FOLLOW_MOVING_UGV
                else self._ugv_motion_envelope()
            ),
            docking_separation_m=None if separation is None else float(separation),
        )

    def _ugv_motion_envelope(self) -> bool:
        pose_fresh = (
            self.ugv_map_pose_time > 0.0
            and time.monotonic() - self.ugv_map_pose_time
            <= self.ugv_map_pose_timeout
        )
        if not pose_fresh:
            return False
        return moving_deck_envelope(
            yaw_rad=self.ugv_yaw,
            yaw_rate_rps=self.ugv_yaw_rate,
            target_yaw_rad=self.moving_ugv[2],
            max_yaw_error_rad=self.moving_dock_max_yaw_error,
            max_yaw_rate_rps=self.moving_dock_max_yaw_rate,
        )

    def _ugv_dock_entry_envelope_raw(self) -> bool:
        pose_fresh = (
            self.ugv_map_pose_time > 0.0
            and time.monotonic() - self.ugv_map_pose_time
            <= self.ugv_map_pose_timeout
        )
        if not pose_fresh:
            return False
        return moving_deck_envelope(
            yaw_rad=self.ugv_yaw,
            yaw_rate_rps=self.ugv_yaw_rate,
            target_yaw_rad=self.moving_ugv[2],
            max_yaw_error_rad=self.moving_dock_entry_max_yaw_error,
            max_yaw_rate_rps=self.moving_dock_entry_max_yaw_rate,
        )

    def _update_ugv_dock_entry_envelope(self, now: float) -> None:
        raw = (
            self.state == MissionState.FOLLOW_MOVING_UGV
            and self._ugv_dock_entry_envelope_raw()
        )
        self.ugv_dock_entry_envelope_since = update_sustained_since(
            raw,
            self.ugv_dock_entry_envelope_since,
            now,
        )

    def _ugv_dock_entry_envelope_ready(self) -> bool:
        now = time.monotonic()
        raw = self._ugv_dock_entry_envelope_raw()
        return sustained_for(
            raw,
            self.ugv_dock_entry_envelope_since,
            now,
            self.moving_dock_entry_hold,
        )

    def _ride_remaining_distance(self):
        pose_fresh = (
            self.ugv_map_pose_time > 0.0
            and time.monotonic() - self.ugv_map_pose_time
            <= self.ugv_map_pose_timeout
        )
        if self.ugv_map_position is None or not pose_fresh:
            return None
        return math.hypot(
            self.ride_ugv[0] - self.ugv_map_position[0],
            self.ride_ugv[1] - self.ugv_map_position[1],
        )

    def _update_ugv_progress_watchdog(self, now: float) -> bool:
        monitored = self.state in (
            MissionState.PARALLEL_TRANSIT,
            MissionState.FOLLOW_MOVING_UGV,
            MissionState.DOCK_MOVING,
            MissionState.RIDE_AND_DECELERATE,
        ) and self.ugv_goal_status in ("sending", "executing")
        if not monitored:
            self.ugv_progress_anchor = None
            self.ugv_progress_since = None
            self.ugv_progress_stalled = False
            return False
        pose_fresh = (
            self.ugv_map_position is not None
            and self.ugv_map_pose_time > 0.0
            and now - self.ugv_map_pose_time <= self.ugv_map_pose_timeout
        )
        position = self.ugv_map_position if pose_fresh else None
        (
            self.ugv_progress_anchor,
            self.ugv_progress_since,
            self.ugv_progress_stalled,
        ) = progress_watchdog_step(
            position_xy=position,
            anchor_xy=self.ugv_progress_anchor,
            anchor_since_s=self.ugv_progress_since,
            now_s=now,
            minimum_progress_m=self.ugv_progress_min_distance,
            timeout_s=self.ugv_progress_timeout * self.timeout_scale,
        )
        return self.ugv_progress_stalled

    def _completion_stop_raw(self) -> bool:
        return bool(
            self.state == MissionState.RIDE_AND_DECELERATE
            and self.ugv_goal_done
            and abs(float(self.ugv_measured_speed)) <= self.completion_stopped_speed
        )

    def _update_completion_stop_hold(self, now: float) -> None:
        self.completion_stopped_since = update_sustained_since(
            self._completion_stop_raw(),
            self.completion_stopped_since,
            now,
        )

    def _completion_stop_ready(self, now: Optional[float] = None) -> bool:
        current = time.monotonic() if now is None else float(now)
        return sustained_for(
            self._completion_stop_raw(),
            self.completion_stopped_since,
            current,
            self.completion_stopped_hold,
        )

    def _transition(self, target: MissionState, reason: str) -> None:
        previous = self.state
        self.state = target
        # Close the final chassis authority gate synchronously with every
        # transition into a non-driving state.  Do not wait for the slower
        # state-action loop or rely on Nav2's last zero command.
        if not mission_state_allows_ugv_motion(target):
            self._set_ugv_speed_scale(0.0)
        self.state_started = time.monotonic()
        self.reason = reason
        self.transition_count += 1
        self.action_dispatched_for_state = None
        self.last_arm_request = 0.0
        self.last_arm_request_wall = 0.0
        self.last_arm_ack_wall = 0.0
        self.arm_retry_not_before = 0.0
        self.last_arm_ack_outcome = None
        self.arm_request_attempts = 0
        self.last_disarm_request = 0.0
        self.disarm_request_attempts = 0
        self.last_takeoff_request = 0.0
        self.last_takeoff_request_wall = 0.0
        self.last_takeoff_ack_wall = 0.0
        self.takeoff_retry_not_before = 0.0
        self.last_takeoff_ack_outcome = None
        self.takeoff_request_attempts = 0
        self.last_land_request = 0.0
        self.land_request_attempts = 0
        self.ugv_dock_entry_envelope_since = 0.0
        self.ugv_progress_anchor = None
        self.ugv_progress_since = None
        self.ugv_progress_stalled = False
        self.completion_stopped_since = 0.0
        event = String()
        event.data = json.dumps(
            {
                "schema_version": "1.0",
                "time_s": round(self.state_started, 3),
                "from": previous.value,
                "to": target.value,
                "reason": reason,
            },
            sort_keys=True,
        )
        self.event_publisher.publish(event)
        self.get_logger().info(f"Mission {previous.value} -> {target.value}: {reason}")

    def _abort(
        self,
        reason: str,
        terminal_state: MissionState = MissionState.ABORTED,
    ) -> None:
        if self.state in (
            MissionState.COMPLETE,
            MissionState.ABORTED,
            MissionState.FAULT,
        ):
            return
        self.cancel_ugv_goal()
        self._set_controller(self.nav_enable_client, False)
        self._set_controller(self.docking_enable_client, False)
        self._set_controller(self.velocity_forward_enable_client, False)
        self._set_ugv_speed_scale(0.0)
        # Never change a physical latch as a generic abort side effect. Keeping
        # an already attached aircraft restrained is the fail-safe condition;
        # release requires a dedicated state with landed/disarmed guards.
        if self.simulation_lifecycle:
            if float(self.telemetry.get("relative_alt_m") or 0.0) > 0.8:
                self._trigger(self.land_client)
            elif (
                self.telemetry.get("landed") is True
                and bool(self.telemetry.get("armed", False))
            ):
                # A fault before lift-off must not leave an armed aircraft on
                # the pad. This uses the normal disarm path; force-arm/disarm
                # remains prohibited.
                self._set_controller(self.arm_client, False)
        self._transition(terminal_state, reason)

    def _drive_lifecycle(self, arm: bool = False) -> None:
        if not self.simulation_lifecycle:
            return
        self._set_controller(self.velocity_forward_enable_client, False)
        self._set_controller(self.guided_client, True)
        now = time.monotonic()
        if (
            arm
            and self.arm_request_attempts < self.arm_max_attempts
            and now - self.last_arm_request >= self.arm_retry
            and now >= self.arm_retry_not_before
            and self.telemetry.get("command_enabled") is False
            and self.arm_client.service_is_ready()
        ):
            self.last_arm_request_wall = time.time()
            self._set_controller(self.arm_client, arm)
            self.last_arm_request = now
            self.arm_request_attempts += 1

    def _observe_arm_command_ack(self, now: float) -> None:
        """Pace retries from the FCU ACK, not ROS service submission.

        A failed ArduPilot arm check can arrive before the slower SYS_STATUS
        pre-arm bit drops. Without this ACK guard every bounded retry can be
        consumed inside that stale-telemetry window.
        """

        if self.last_arm_request_wall <= 0.0:
            return
        acknowledgement = self.telemetry.get("last_command_ack")
        outcome = mavlink_command_ack_outcome(acknowledgement, 400)
        if outcome is None:
            return
        try:
            received_wall = float(acknowledgement.get("received_at_s"))
        except (AttributeError, TypeError, ValueError):
            return
        if (
            received_wall < self.last_arm_request_wall
            or received_wall <= self.last_arm_ack_wall
        ):
            return
        self.last_arm_ack_wall = received_wall
        self.last_arm_ack_outcome = outcome
        retry_delay = (
            self.arm_failure_cooldown
            if outcome == "failed"
            else self.arm_confirmation_timeout
        )
        self.arm_retry_not_before = max(
            self.arm_retry_not_before,
            acknowledged_retry_deadline(
                now_s=now,
                wall_now_s=time.time(),
                ack_wall_s=received_wall,
                delay_s=retry_delay,
            ),
        )

    def _observe_takeoff_command_ack(self, now: float) -> None:
        """Treat TAKEOFF as an acknowledged operation, not a fire-and-repeat pulse."""

        if self.last_takeoff_request_wall <= 0.0:
            return
        acknowledgement = self.telemetry.get("last_command_ack")
        outcome = mavlink_command_ack_outcome(acknowledgement, 22)
        if outcome is None:
            return
        try:
            received_wall = float(acknowledgement.get("received_at_s"))
        except (AttributeError, TypeError, ValueError):
            return
        if (
            received_wall < self.last_takeoff_request_wall
            or received_wall <= self.last_takeoff_ack_wall
        ):
            return
        self.last_takeoff_ack_wall = received_wall
        self.last_takeoff_ack_outcome = outcome
        retry_delay = (
            self.takeoff_failure_cooldown
            if outcome == "failed"
            else self.takeoff_confirmation_timeout
        )
        self.takeoff_retry_not_before = max(
            self.takeoff_retry_not_before,
            acknowledged_retry_deadline(
                now_s=now,
                wall_now_s=time.time(),
                ack_wall_s=received_wall,
                delay_s=retry_delay,
            ),
        )

    def _drive_terminal_safety(self, now: float) -> None:
        """Keep terminal fault handling active until normal landing is confirmed."""

        self._set_ugv_speed_scale(0.0)
        if not self.simulation_lifecycle or not bool(
            self.telemetry.get("armed", False)
        ):
            return
        if self.telemetry.get("landed") is True:
            if (
                self.disarm_request_attempts < self.arm_max_attempts
                and now - self.last_disarm_request >= self.arm_retry
                and self.arm_client.service_is_ready()
            ):
                self._set_controller(self.arm_client, False)
                self.last_disarm_request = now
                self.disarm_request_attempts += 1
            return
        if (
            str(self.telemetry.get("mode", "")).upper() != "LAND"
            and self.land_request_attempts < self.land_max_attempts
            and now - self.last_land_request >= self.land_retry
            and self.land_client.service_is_ready()
        ):
            self._trigger(self.land_client)
            self.last_land_request = now
            self.land_request_attempts += 1

    def _drive_state_actions(self, now: float) -> None:
        if now - self.last_action_drive < 1.0:
            return
        self.last_action_drive = now
        state = self.state
        # Reassert the invariant as a heartbeat so a stale or restarted
        # consumer cannot inherit motion permission while landing or latched.
        if not mission_state_allows_ugv_motion(state):
            self._set_ugv_speed_scale(0.0)
        if state in (
            MissionState.RELEASE_REMOTE_DOCK,
            MissionState.RELEASE_FOR_TRANSIT,
            MissionState.RELEASE_FOR_FOLLOW,
        ):
            self._set_controller(self.velocity_forward_enable_client, False)
            self.detach_publisher.publish(Empty())
            self._set_controller(self.docking_enable_client, False)
        elif state in (
            MissionState.ARM_INITIAL,
            MissionState.ARM_FOR_TRANSIT,
            MissionState.ARM_FOR_FOLLOW,
        ):
            # A single good SYS_STATUS sample is not enough to authorize an
            # arm transition. Both the autopilot and independent supervisor
            # must remain continuously ready for their configured hold time.
            self._observe_arm_command_ack(now)
            if self._preflight_ready() and self._system_ready():
                self._drive_lifecycle(arm=True)
            else:
                self._set_controller(self.velocity_forward_enable_client, False)
        elif state in (
            MissionState.TAKEOFF_INITIAL,
            MissionState.TAKEOFF_FOR_TRANSIT,
            MissionState.TAKEOFF_FOR_FOLLOW,
        ):
            self._set_controller(self.velocity_forward_enable_client, False)
            self._observe_takeoff_command_ack(now)
            if (
                self.telemetry.get("landed") is True
                and bool(self.telemetry.get("armed", False))
                and self.telemetry.get("command_enabled") is False
                and bool(self.perception.get("healthy", False))
                and self.takeoff_request_attempts < self.takeoff_max_attempts
                and now - self.last_takeoff_request >= self.takeoff_retry
                and now >= self.takeoff_retry_not_before
                and self.takeoff_client.service_is_ready()
            ):
                self.last_takeoff_request_wall = time.time()
                self._trigger(self.takeoff_client)
                self.last_takeoff_request = now
                self.takeoff_request_attempts += 1
        elif state == MissionState.NAVIGATE_TO_START_DOCK:
            self._set_controller(self.velocity_forward_enable_client, True)
            self._set_controller(self.docking_enable_client, False)
            self._set_controller(self.nav_enable_client, True)
            self._trigger(self.gimbal_down_client)
            self._publish_uav_goal(
                self.initial_ugv[0], self.initial_ugv[1], self.start_dock_altitude
            )
        elif state in (MissionState.DOCK_AT_START, MissionState.DOCK_STOPPED):
            self._set_controller(self.velocity_forward_enable_client, True)
            self._set_controller(self.nav_enable_client, False)
            self._set_mode("stopped")
            self._set_controller(self.docking_enable_client, True)
            self._trigger(self.gimbal_down_client)
        elif state == MissionState.PARALLEL_TRANSIT:
            self._set_ugv_speed_scale(1.0)
            self._set_controller(self.velocity_forward_enable_client, True)
            self._set_controller(self.docking_enable_client, False)
            self._set_controller(self.nav_enable_client, True)
            self._trigger(self.gimbal_center_client)
            self._publish_uav_goal(*self.transit_uav)
            if self.action_dispatched_for_state != state:
                self.send_ugv_goal(*self.transit_ugv)
                if self.ugv_goal_status in ("sending", "executing"):
                    self.action_dispatched_for_state = state
        elif state == MissionState.FOLLOW_MOVING_UGV:
            self._set_ugv_speed_scale(self.follow_ugv_speed_scale)
            self._set_controller(self.velocity_forward_enable_client, True)
            self._set_controller(self.nav_enable_client, False)
            self._set_mode("follow")
            self._set_controller(self.docking_enable_client, True)
            self._trigger(self.gimbal_down_client)
            if self.action_dispatched_for_state != state:
                self.send_ugv_goal(*self.moving_ugv)
                if self.ugv_goal_status in ("sending", "executing"):
                    self.action_dispatched_for_state = state
        elif state == MissionState.DOCK_MOVING:
            self._set_ugv_speed_scale(self.docking_ugv_speed_scale)
            self._set_controller(self.velocity_forward_enable_client, True)
            self._set_mode("moving")
            self._set_controller(self.docking_enable_client, True)
        elif state in (
            MissionState.LATCH_AT_START,
            MissionState.LATCH_STOPPED,
            MissionState.LATCH_MOVING,
        ):
            if state == MissionState.LATCH_MOVING:
                self._set_ugv_speed_scale(self.docking_ugv_speed_scale)
            self._set_controller(self.velocity_forward_enable_client, False)
            self._set_controller(self.docking_enable_client, False)
            # LAND must establish the flight-controller contact state. The
            # bridge independently authorizes or rejects this operation for
            # the active profile; arm/takeoff permissions stay separate.
            if (
                str(self.telemetry.get("mode", "")).upper() != "LAND"
                and self.land_request_attempts < self.land_max_attempts
                and now - self.last_land_request >= self.land_retry
                and self.land_client.service_is_ready()
            ):
                self._trigger(self.land_client)
                self.last_land_request = now
                self.land_request_attempts += 1
            if self.simulation_lifecycle:
                # A normal disarm request is allowed only after a positive
                # landed indication; force disarm remains prohibited.
                if (
                    self.telemetry.get("landed") is True
                    and bool(self.telemetry.get("armed", False))
                    and self.disarm_request_attempts < self.arm_max_attempts
                    and now - self.last_disarm_request >= self.arm_retry
                    and self.arm_client.service_is_ready()
                ):
                    self._set_controller(self.arm_client, False)
                    self.last_disarm_request = now
                    self.disarm_request_attempts += 1
            if dock_attach_authorized(
                state,
                armed=bool(self.telemetry.get("armed", False)),
                landed=self.telemetry.get("landed"),
                autopilot_mode=str(self.telemetry.get("mode", "")),
                altitude_m=float(self.telemetry.get("relative_alt_m") or 0.0),
                moving_capture_max_altitude_m=self.moving_capture_max_altitude,
            ):
                self.attach_publisher.publish(Empty())
        elif state == MissionState.RIDE_AND_DECELERATE:
            remaining = self._ride_remaining_distance()
            self._set_ugv_speed_scale(
                0.0
                if remaining is None
                else distance_speed_scale(
                    self.docking_ugv_speed_scale,
                    self.ride_final_ugv_speed_scale,
                    remaining,
                    self.ride_slowdown_distance,
                )
            )
            self._set_controller(self.velocity_forward_enable_client, False)
            self._set_controller(self.nav_enable_client, False)
            self._set_controller(self.docking_enable_client, False)
            if self.action_dispatched_for_state != state:
                self.send_ugv_goal(*self.ride_ugv)
                if self.ugv_goal_status in ("sending", "executing"):
                    self.action_dispatched_for_state = state

    def control_tick(self) -> None:
        now = time.monotonic()
        self._refresh_ugv_map_pose_from_tf()
        self._update_ugv_dock_entry_envelope(now)
        if (
            self.auto_start
            and self.state == MissionState.IDLE
            and now - self.boot_time >= self.auto_start_delay
        ):
            request = Trigger.Request()
            response = Trigger.Response()
            self.on_start(request, response)
        if self.state in (MissionState.ABORTED, MissionState.FAULT):
            self._drive_terminal_safety(now)
            return
        if self.state in (MissionState.IDLE, MissionState.COMPLETE) or self.paused:
            return

        if (
            self.state
            in (
                MissionState.TAKEOFF_INITIAL,
                MissionState.TAKEOFF_FOR_TRANSIT,
                MissionState.TAKEOFF_FOR_FOLLOW,
            )
            and self.last_takeoff_request > 0.0
            and now - self.last_takeoff_request >= self.takeoff_disarm_grace
            and not bool(self.telemetry.get("armed", False))
        ):
            self._abort("uav_disarmed_before_takeoff", MissionState.FAULT)
            return

        if self.state in (
            MissionState.PARALLEL_TRANSIT,
            MissionState.FOLLOW_MOVING_UGV,
            MissionState.DOCK_MOVING,
            MissionState.RIDE_AND_DECELERATE,
        ) and navigation_goal_failed(self.ugv_goal_status):
            self._abort(
                f"ugv_navigation_failed_{self.ugv_goal_status.split(':', 1)[0]}",
                MissionState.FAULT,
            )
            return
        if self._update_ugv_progress_watchdog(now):
            self._abort("ugv_navigation_progress_stalled", MissionState.FAULT)
            return
        self._update_completion_stop_hold(now)
        if self.state == MissionState.DOCK_MOVING and not self._ugv_motion_envelope():
            self._transition(
                MissionState.FOLLOW_MOVING_UGV,
                "moving_deck_envelope_lost_go_around",
            )
            return
        if self.state == MissionState.LATCH_MOVING and not self._ugv_motion_envelope():
            self._abort("moving_deck_envelope_lost_during_capture", MissionState.FAULT)
            return

        self._drive_state_actions(now)
        elapsed = now - self.state_started
        target = next_state(
            self.state,
            elapsed,
            self._facts(),
            timeout_scale=self.timeout_scale,
            timeout_overrides_s=self.state_timeout_overrides,
        )
        if target == MissionState.FAULT:
            failed_state = self.state.value
            self._abort(f"state_timeout_{failed_state}", MissionState.FAULT)
            return
        if target != self.state:
            self._transition(target, "guard_satisfied")
            if target in (
                MissionState.PARALLEL_TRANSIT,
                MissionState.FOLLOW_MOVING_UGV,
                MissionState.RIDE_AND_DECELERATE,
            ):
                self.ugv_goal_done = False
                self.ugv_goal_status = "pending"
            if target == MissionState.COMPLETE:
                self._set_ugv_speed_scale(0.0)
                self._set_controller(self.nav_enable_client, False)
                self._set_controller(self.docking_enable_client, False)
                self._set_controller(self.velocity_forward_enable_client, False)

    def publish_status(self) -> None:
        now = time.monotonic()
        self._refresh_ugv_map_pose_from_tf()
        self._update_ugv_dock_entry_envelope(now)
        ride_remaining = self._ride_remaining_distance()
        completion_stop_ready = self._completion_stop_ready(now)
        completion_stop_observed = (
            0.0
            if self.completion_stopped_since <= 0.0
            else max(0.0, now - self.completion_stopped_since)
        )
        state_timeout = self.state_timeout_overrides.get(
            self.state, STATE_TIMEOUTS.get(self.state)
        )
        message = String()
        message.data = json.dumps(
            {
                "schema_version": "1.0",
                "active": self.state not in (
                    MissionState.IDLE,
                    MissionState.COMPLETE,
                    MissionState.ABORTED,
                    MissionState.FAULT,
                ),
                "state": self.state.value,
                "reason": self.reason,
                "paused": self.paused,
                "elapsed_s": 0.0 if self.started_at == 0.0 else round(now - self.started_at, 1),
                "state_elapsed_s": round(now - self.state_started, 1),
                "state_timeout_s": (
                    None
                    if state_timeout is None
                    else round(float(state_timeout) * self.timeout_scale, 1)
                ),
                "transitions": self.transition_count,
                "ugv_goal_status": self.ugv_goal_status,
                "ugv_goal_done": self.ugv_goal_done,
                "ugv_progress_watchdog": {
                    "stalled": self.ugv_progress_stalled,
                    "age_s": (
                        None
                        if self.ugv_progress_since is None
                        else round(max(0.0, now - self.ugv_progress_since), 3)
                    ),
                    "minimum_progress_m": round(self.ugv_progress_min_distance, 3),
                    "timeout_s": round(
                        self.ugv_progress_timeout * self.timeout_scale, 3
                    ),
                },
                "completion_stop_hold": {
                    "raw": self._completion_stop_raw(),
                    "ready": completion_stop_ready,
                    "observed_s": round(completion_stop_observed, 3),
                    "required_s": round(self.completion_stopped_hold, 3),
                    "maximum_speed_mps": round(self.completion_stopped_speed, 3),
                },
                "ugv_speed_scale": round(self.ugv_speed_scale, 3),
                "ugv_nav2_speed_limit_pct": round(self.ugv_speed_scale * 100.0, 1),
                "ugv_safety_gate_open": self.ugv_speed_scale > 0.0,
                "ugv_measured_speed_mps": round(self.ugv_measured_speed, 3),
                "ugv_map_position": (
                    None
                    if self.ugv_map_position is None
                    else [round(value, 3) for value in self.ugv_map_position]
                ),
                "ugv_map_pose_age_s": (
                    None
                    if self.ugv_map_pose_time == 0.0
                    else round(now - self.ugv_map_pose_time, 3)
                ),
                "ugv_map_pose_source": self.ugv_map_pose_source,
                "ugv_map_transform_age_s": (
                    None
                    if self.ugv_map_transform_age is None
                    else round(self.ugv_map_transform_age, 3)
                ),
                "ugv_yaw_rad": (
                    None if self.ugv_yaw is None else round(self.ugv_yaw, 3)
                ),
                "ugv_yaw_rate_rps": round(self.ugv_yaw_rate, 3),
                "ugv_motion_envelope": self._ugv_motion_envelope(),
                "ugv_dock_entry_envelope": {
                    "raw": self._ugv_dock_entry_envelope_raw(),
                    "ready": self._ugv_dock_entry_envelope_ready(),
                    "hold_required_s": round(self.moving_dock_entry_hold, 2),
                    "hold_observed_s": round(
                        0.0
                        if self.ugv_dock_entry_envelope_since == 0.0
                        else max(0.0, now - self.ugv_dock_entry_envelope_since),
                        2,
                    ),
                },
                "ride_remaining_distance_m": (
                    None if ride_remaining is None else round(ride_remaining, 3)
                ),
                "ugv_moving_confirmed": (
                    self.ugv_moving_since > 0.0
                    and now - self.ugv_moving_since >= self.moving_ugv_confirm
                ),
                "dock_detached": self.dock_detached,
                "dock_attached_age_s": (
                    None
                    if self.dock_detached is not False
                    or self.dock_attached_since == 0.0
                    else round(max(0.0, now - self.dock_attached_since), 3)
                ),
                "last_uav_goal": self.last_uav_goal,
                "telemetry": {
                    "connected": bool(self.telemetry.get("connected", False)),
                    "flight_ready": self._facts().flight_ready,
                    "armed": bool(self.telemetry.get("armed", False)),
                    "mode": self.telemetry.get("mode", "UNKNOWN"),
                    "altitude_m": self.telemetry.get("relative_alt_m"),
                },
                "lifecycle_command_attempts": {
                    "arm": [self.arm_request_attempts, self.arm_max_attempts],
                    "takeoff": [
                        self.takeoff_request_attempts,
                        self.takeoff_max_attempts,
                    ],
                    "land": [self.land_request_attempts, self.land_max_attempts],
                    "disarm": [self.disarm_request_attempts, self.arm_max_attempts],
                    "last_arm_ack_outcome": self.last_arm_ack_outcome,
                    "arm_retry_in_s": round(
                        max(0.0, self.arm_retry_not_before - now), 2
                    ),
                    "last_takeoff_ack_outcome": self.last_takeoff_ack_outcome,
                    "takeoff_retry_in_s": round(
                        max(0.0, self.takeoff_retry_not_before - now), 2
                    ),
                },
                "navigation": self.navigation,
                "docking": self.docking,
                "perception_healthy": bool(self.perception.get("healthy", False)),
                "system_ready": self._system_ready(),
                "readiness_hold": {
                    "system_required_s": round(self.system_ready_hold, 2),
                    "system_observed_s": round(
                        0.0
                        if self.system_ready_since == 0.0
                        else max(0.0, now - self.system_ready_since),
                        2,
                    ),
                    "preflight_required_s": round(self.preflight_ready_hold, 2),
                    "preflight_observed_s": round(
                        0.0
                        if self.preflight_ready_since == 0.0
                        else max(0.0, now - self.preflight_ready_since),
                        2,
                    ),
                },
                "system_emergency_stop": self.emergency_stop,
                "mission_plan": {
                    "id": self.mission_plan_id,
                    "commissioned": mission_plan_is_commissioned(
                        simulation_lifecycle=self.simulation_lifecycle,
                        validated=self.mission_plan_validated,
                        plan_id=self.mission_plan_id,
                    ),
                    "simulation": self.simulation_lifecycle,
                },
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AirGroundMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            run_shutdown_action(
                lambda: (node.cancel_ugv_goal(), node._set_ugv_speed_scale(0.0))
            )
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
