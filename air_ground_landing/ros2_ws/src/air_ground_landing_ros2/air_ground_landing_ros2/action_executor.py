"""The sole ROS 2 owner of flight-action MAVROS writes.

Commands arrive as JSON on ``/landing/action/request``.  The node exposes one
external action lifecycle while keeping mode acknowledgement and sensor timing
private.  Its outputs are disabled by default.
"""

from __future__ import annotations

import json
import math
import os
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import (
    EstimatorStatus,
    ExtendedState,
    HomePosition,
    Mavlink,
    PositionTarget,
    RCIn,
    State,
)
from mavros_msgs.srv import CommandLong, SetMode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Range
from std_msgs.msg import String

from air_ground_landing.action_execution import (
    ActionCommand,
    ActionExecutor as ActionLifecycle,
    ActionKind,
    ActionRequest,
    ActionState,
    GUIDED_ACTIONS,
    VehicleSnapshot,
)
from air_ground_landing.guided_execution import (
    PilotSessionGate,
    RcAuthorizationGate,
    RcGateConfig,
)
from air_ground_landing.mavlink_ekf import report_health
from air_ground_landing_ros2.action_peripherals import ActionPeripherals, PERIPHERAL_DEFAULTS
from air_ground_landing.landing_alignment import LandingAlignment


MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_DO_AUX_FUNCTION = 218
MOTOR_EMERGENCY_STOP_AUX_FUNCTION = 31


class ActionExecutorNode(ActionPeripherals, Node):
    def __init__(self) -> None:
        super().__init__("action_executor")
        self._declare_parameters()
        self.environment = str(self.get_parameter("environment").value).strip().lower()
        if self.environment not in {"offline", "sitl", "hardware"}:
            raise ValueError("environment must be offline, sitl or hardware")
        approved = self.environment == "sitl" or (
            self.environment == "hardware"
            and bool(self.get_parameter("flight_use_approved").value)
        )
        self.allow_mode = approved and bool(self.get_parameter("allow_mode_change").value)
        self.allow_setpoint = approved and bool(self.get_parameter("allow_setpoint_output").value)
        self.allow_disarm = approved and bool(self.get_parameter("allow_disarm_output").value)
        self.allow_landing_disarm = approved and bool(
            self.get_parameter("allow_landing_disarm_output").value
        )
        self.allow_emergency_stop = approved and bool(
            self.get_parameter("allow_emergency_stop_output").value
        )
        self.require_rc = bool(self.get_parameter("require_rc_authorization").value)
        self.rc_flight_flow_enabled = bool(
            self.get_parameter("rc_flight_flow_enabled").value
        )
        if self.environment == "hardware" and not self.require_rc:
            raise ValueError("hardware execution requires RC authorization")
        self.output_enabled = self.allow_mode and self.allow_setpoint
        self.guided_mode = str(self.get_parameter("guided_mode").value).strip().upper()
        self.land_mode = str(self.get_parameter("land_mode").value).strip().upper()
        self.entry_modes = {
            str(value).strip().upper()
            for value in self.get_parameter("entry_modes").value
        }
        self.state_maximum_age_s = float(self.get_parameter("state_maximum_age_s").value)
        self.ekf_maximum_age_s = float(self.get_parameter("ekf_maximum_age_s").value)
        self.pose_maximum_age_s = float(self.get_parameter("pose_maximum_age_s").value)
        self.velocity_maximum_age_s = float(self.get_parameter("velocity_maximum_age_s").value)
        self.candidate_maximum_age_s = float(self.get_parameter("candidate_maximum_age_s").value)
        self.landing_alignment_maximum_age_s = float(self.get_parameter("landing_alignment_maximum_age_s").value)
        self.range_maximum_age_s = float(self.get_parameter("range_maximum_age_s").value)
        self.landing_target_maximum_age_s = float(
            self.get_parameter("landing_target_maximum_age_s").value
        )
        self.mode_retry_s = float(self.get_parameter("mode_retry_s").value)
        self.mode_heartbeat_ack_timeout_s = float(
            self.get_parameter("mode_heartbeat_ack_timeout_s").value
        )
        self.command_retry_s = float(self.get_parameter("command_retry_s").value)
        self.landing_switch_channel = int(
            self.get_parameter("landing_switch_channel").value
        )
        self.landing_switch_off_below_pwm = int(
            self.get_parameter("landing_switch_off_below_pwm").value
        )
        self.landing_switch_on_above_pwm = int(
            self.get_parameter("landing_switch_on_above_pwm").value
        )
        self.landing_switch_maximum_age_s = float(
            self.get_parameter("landing_switch_maximum_age_s").value
        )
        if not self.guided_mode or not self.land_mode or not self.entry_modes:
            raise ValueError("guided_mode, land_mode and entry_modes are required")
        if not all(math.isfinite(v) and v > 0.0 for v in (
            self.state_maximum_age_s,
            self.ekf_maximum_age_s,
            self.pose_maximum_age_s,
            self.velocity_maximum_age_s,
            self.candidate_maximum_age_s,
            self.landing_alignment_maximum_age_s,
            self.range_maximum_age_s,
            self.landing_target_maximum_age_s,
            self.mode_retry_s,
            self.mode_heartbeat_ack_timeout_s,
            self.command_retry_s,
            self.landing_switch_maximum_age_s,
        )):
            raise ValueError("all age and retry parameters must be finite and positive")
        if self.landing_switch_channel < 1:
            raise ValueError("landing_switch_channel must be positive")
        if not (
            800
            <= self.landing_switch_off_below_pwm
            < self.landing_switch_on_above_pwm
            <= 2200
        ):
            raise ValueError("landing switch PWM thresholds are invalid")

        self._lock_handle = self._acquire_single_writer_lock(
            str(self.get_parameter("single_writer_lock_path").value)
        )
        self.lifecycle = ActionLifecycle(
            position_tolerance_m=float(self.get_parameter("position_tolerance_m").value),
            speed_tolerance_mps=float(self.get_parameter("speed_tolerance_mps").value),
            yaw_tolerance_rad=math.radians(float(self.get_parameter("yaw_tolerance_deg").value)),
            completion_dwell_s=float(self.get_parameter("completion_dwell_s").value),
            follow_loss_grace_s=float(self.get_parameter("follow_loss_grace_s").value),
            maximum_horizontal_speed_mps=float(
                self.get_parameter("maximum_horizontal_speed_mps").value
            ),
            maximum_vertical_speed_mps=float(
                self.get_parameter("maximum_vertical_speed_mps").value
            ),
            maximum_yaw_rate_rad_s=math.radians(
                float(self.get_parameter("maximum_yaw_rate_deg_s").value)
            ),
            maximum_horizontal_acceleration_mps2=float(
                self.get_parameter("maximum_horizontal_acceleration_mps2").value
            ),
            maximum_vertical_acceleration_mps2=float(
                self.get_parameter("maximum_vertical_acceleration_mps2").value
            ),
            guided_mode=self.guided_mode,
            land_mode=self.land_mode,
            fallback_mode=str(self.get_parameter("fallback_mode").value),
            land_recovery_height_m=float(
                self.get_parameter("land_recovery_height_m").value
            ),
            land_recovery_timeout_s=float(
                self.get_parameter("land_recovery_timeout_s").value
            ),
            land_reacquire_dwell_s=float(
                self.get_parameter("land_reacquire_dwell_s").value
            ),
            land_guided_descent_mps=float(self.get_parameter("land_guided_descent_mps").value),
            land_yaw_tolerance_rad=math.radians(float(self.get_parameter("land_yaw_tolerance_deg").value)),
            land_yaw_gain_per_s=float(self.get_parameter("land_yaw_gain_per_s").value),
            land_maximum_yaw_rate_rad_s=math.radians(float(self.get_parameter("land_maximum_yaw_rate_deg_s").value)),
            land_yaw_alignment_dwell_s=float(self.get_parameter("land_yaw_alignment_dwell_s").value),
            land_yaw_alignment_maximum_gap_s=self.landing_alignment_maximum_age_s,
            land_yaw_alignment_rate_tolerance_rad_s=math.radians(float(
                self.get_parameter("land_yaw_alignment_rate_tolerance_deg_s").value)),
        )
        self.rc_gate = RcAuthorizationGate(RcGateConfig(
            channel=int(self.get_parameter("rc_channel").value),
            abort_below_pwm=int(self.get_parameter("rc_abort_below_pwm").value),
            authorize_above_pwm=int(self.get_parameter("rc_authorize_above_pwm").value),
            maximum_age_s=float(self.get_parameter("rc_maximum_age_s").value),
        ))
        self.pilot_session = PilotSessionGate(
            low_dwell_s=float(self.get_parameter("rc_rearm_dwell_s").value)
        )

        self.vehicle_state = State()
        self.vehicle_state.connected = False
        self.state_received_s: Optional[float] = None
        self.pose: Optional[PoseStamped] = None
        self.pose_received_s: Optional[float] = None
        self.velocity: Optional[TwistStamped] = None
        self.velocity_received_s: Optional[float] = None
        self.extended: Optional[ExtendedState] = None
        self.extended_received_s: Optional[float] = None
        self.estimator_healthy = False
        self.estimator_received_s: Optional[float] = None
        self.ekf_report_received_s: Optional[float] = None
        self.ekf_report_count = 0
        self.home_set = False
        self.home_received_s: Optional[float] = None
        self.rc_channels: Optional[tuple[int, ...]] = None
        self.rc_received_s: Optional[float] = None
        self.candidate: Optional[PositionTarget] = None
        self.candidate_received_s: Optional[float] = None
        self.target_healthy = False
        self.target_aligned = False
        self.target_status_received_s = None
        self.landing_alignment = None
        self.range_m = math.nan
        self.range_received_s: Optional[float] = None
        self.landing_target_stream_healthy = False
        self.landing_target_output_enabled = False
        self.landing_target_status_received_s: Optional[float] = None
        self.mode_future = None
        self.mode_request_action_id: Optional[str] = None
        self.mode_request_s = -math.inf
        self.owned_mode_deadlines: dict[str, float] = {}
        self.auto_land_inhibited = False
        self.command_future = None
        self.command_action_id: Optional[str] = None
        self.command_kind: Optional[str] = None
        self.command_request_s = -math.inf
        self.last_status = None
        self.auto_action_sequence = 0
        self.last_auto_follow_attempt_s = -math.inf
        self.landing_switch_state = "MISSING"
        self.landing_switch_pwm: Optional[int] = None

        self.setpoint_publisher = self.create_publisher(
            PositionTarget, str(self.get_parameter("mavros_output_topic").value), 10
        )
        self.preview_publisher = self.create_publisher(
            PositionTarget, str(self.get_parameter("preview_topic").value), 10
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.mode_client = self.create_client(
            SetMode, str(self.get_parameter("mavros_set_mode_service").value)
        )
        self.command_client = self.create_client(
            CommandLong, str(self.get_parameter("mavros_command_service").value)
        )
        self._init_peripherals(approved)
        reliable_latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        subscriptions = (
            (String, "request_topic", self._request, 10),
            (String, "cancel_topic", self._cancel, 10),
            (State, "mavros_state_topic", self._state, reliable_latched),
            (RCIn, "mavros_rc_topic", self._rc, qos_profile_sensor_data),
            (PoseStamped, "pose_topic", self._pose, qos_profile_sensor_data),
            (
                TwistStamped,
                "velocity_topic",
                self._velocity,
                qos_profile_sensor_data,
            ),
            (
                ExtendedState,
                "extended_state_topic",
                self._extended,
                qos_profile_sensor_data,
            ),
            (
                EstimatorStatus,
                "estimator_status_topic",
                self._estimator,
                qos_profile_sensor_data,
            ),
            (Mavlink, "ekf_report_topic", self._ekf_report, qos_profile_sensor_data),
            (HomePosition, "home_position_topic", self._home, reliable_latched),
            (PositionTarget, "ibvs_candidate_topic", self._candidate, 10),
            (String, "ibvs_status_topic", self._target_status, 10),
            (
                String,
                "landing_target_status_topic",
                self._landing_target_status,
                10,
            ),
            (Range, "rangefinder_topic", self._range, qos_profile_sensor_data),
        )
        for message_type, parameter, callback, qos in subscriptions:
            self.create_subscription(
                message_type,
                str(self.get_parameter(parameter).value),
                callback,
                qos,
            )
        rate_hz = float(self.get_parameter("output_rate_hz").value)
        if not math.isfinite(rate_hz) or not 2.0 <= rate_hz <= 50.0:
            raise ValueError("output_rate_hz must be in [2, 50]")
        self.create_timer(1.0 / rate_hz, self._tick)

    def _declare_parameters(self) -> None:
        defaults = {
            **PERIPHERAL_DEFAULTS,
            "environment": "offline",
            "flight_use_approved": False,
            "allow_mode_change": False,
            "allow_setpoint_output": False,
            "allow_disarm_output": False,
            "allow_landing_disarm_output": False,
            "allow_emergency_stop_output": False,
            "require_rc_authorization": True,
            "rc_flight_flow_enabled": True,
            "rc_channel": 6,
            "land_guided_descent_mps": 0.10,
            "land_yaw_tolerance_deg": 4.0,
            "land_yaw_gain_per_s": 0.8,
            "land_yaw_alignment_dwell_s": 0.5,
            "land_yaw_alignment_rate_tolerance_deg_s": 3.0,
            "land_maximum_yaw_rate_deg_s": 15.0,
            "landing_alignment_maximum_age_s": 0.3,
            "rc_abort_below_pwm": 1300,
            "rc_authorize_above_pwm": 1800,
            "rc_maximum_age_s": 0.5,
            "rc_rearm_dwell_s": 0.4,
            "landing_switch_channel": 8,
            "landing_switch_off_below_pwm": 1200,
            "landing_switch_on_above_pwm": 1800,
            "landing_switch_maximum_age_s": 0.5,
            "guided_mode": "GUIDED",
            "land_mode": "LAND",
            "fallback_mode": "LOITER",
            "entry_modes": ["ALT_HOLD", "LOITER"],
            "land_recovery_height_m": 0.10,
            "land_recovery_timeout_s": 1.0,
            "land_reacquire_dwell_s": 0.0,
            "position_tolerance_m": 0.08,
            "speed_tolerance_mps": 0.08,
            "yaw_tolerance_deg": 4.0,
            "completion_dwell_s": 0.3,
            "follow_loss_grace_s": 0.5,
            "maximum_horizontal_speed_mps": 0.5,
            "maximum_vertical_speed_mps": 0.2,
            "maximum_yaw_rate_deg_s": 45.0,
            "maximum_horizontal_acceleration_mps2": 0.5,
            "maximum_vertical_acceleration_mps2": 0.3,
            "state_maximum_age_s": 0.5,
            # EKF_STATUS_REPORT can have multi-second gaps on TELEM1 even
            # while the 10 Hz pose and velocity streams remain fresh.
            "ekf_maximum_age_s": 5.0,
            "pose_maximum_age_s": 0.3,
            "velocity_maximum_age_s": 0.3,
            "candidate_maximum_age_s": 0.3,
            "range_maximum_age_s": 0.5,
            "landing_target_maximum_age_s": 0.5,
            "mode_retry_s": 1.0,
            "mode_heartbeat_ack_timeout_s": 2.0,
            "command_retry_s": 1.0,
            "output_rate_hz": 20.0,
            "single_writer_lock_path": "/tmp/uav_action_executor.lock",
            "request_topic": "/landing/action/request",
            "cancel_topic": "/landing/action/cancel",
            "status_topic": "/landing/action/status",
            "preview_topic": "/landing/action/preview",
            "mavros_output_topic": "/mavros/setpoint_raw/local",
            "mavros_state_topic": "/mavros/state",
            "mavros_rc_topic": "/mavros/rc/in",
            "mavros_set_mode_service": "/mavros/set_mode",
            "mavros_command_service": "/mavros/cmd/command",
            "pose_topic": "/mavros/local_position/pose",
            "velocity_topic": "/mavros/local_position/velocity_local",
            "extended_state_topic": "/mavros/extended_state",
            "estimator_status_topic": "/mavros/estimator_status",
            "ekf_report_topic": "/uas1/mavlink_source",
            "home_position_topic": "/mavros/home_position/home",
            "ibvs_candidate_topic": "/landing/ibvs/candidate",
            "ibvs_status_topic": "/landing/ibvs/status",
            "landing_target_status_topic": "/landing/landing_target/status",
            "rangefinder_topic": "/mavros/distance_sensor/rangefinder_pub",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    @staticmethod
    def _now_s() -> float:
        return time.monotonic()

    def _request(self, message: String) -> None:
        now = self._now_s()
        try:
            payload = json.loads(message.data)
            request = ActionRequest.from_mapping(payload)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self._publish_event({
                "state": ActionState.FAILED.value,
                "reason": "REJECTED_MALFORMED_REQUEST",
                "detail": str(exc),
            })
            return
        if (self.output_enabled and self.orphan_recovery.pending
                and request.kind != ActionKind.EMERGENCY_STOP):
            self._publish_event({"state": "FAILED", "reason": "ORPHANED_GUIDED_RECOVERY_PENDING"})
            return
        status = self.lifecycle.start(request, self._snapshot(now))
        self._publish_status(status, now, event="REQUEST")

    def _cancel(self, message: String) -> None:
        value = message.data.strip()
        action_id: Optional[str]
        reason = "TASK_CANCELLED"
        try:
            payload = json.loads(value)
        except (TypeError, ValueError, json.JSONDecodeError):
            action_id = value or None
        else:
            if isinstance(payload, dict):
                action_id = str(payload.get("action_id", "")).strip() or None
                reason = str(payload.get("reason", reason)).strip() or reason
            else:
                action_id = str(payload).strip() or None
        status = self.lifecycle.cancel(action_id, self._now_s(), reason)
        self._publish_status(status, self._now_s(), event="CANCEL")

    def _state(self, message: State) -> None:
        now = self._now_s()
        expected_mode = self._expected_mode()
        self.owned_mode_deadlines = {
            mode: deadline
            for mode, deadline in self.owned_mode_deadlines.items()
            if deadline >= now
        }
        observed_mode = message.mode.strip().upper()
        gate_expected_mode = expected_mode
        if observed_mode in self.owned_mode_deadlines:
            # SetMode replies before State.mode changes.  A newer decision may
            # supersede the request during that gap, so its late heartbeat is
            # still executor-owned and must not be classified as pilot input.
            gate_expected_mode = observed_mode
        override = self.pilot_session.observe_mode(
            message.mode,
            expected_mode=gate_expected_mode,
            now_s=now,
            watch_transition=expected_mode is not None,
        )
        self.vehicle_state = message
        self.state_received_s = now
        if override and (self.lifecycle.active or self.lifecycle.retaining_control):
            self.lifecycle.cancel(
                self.lifecycle.request.action_id,
                now,
                "PILOT_OVERRIDE",
            )
        if not message.connected or not message.armed:
            self.pilot_session.invalidate("DISCONNECTED_OR_DISARMED", now)

    def _rc(self, message: RCIn) -> None:
        self.rc_channels = tuple(int(value) for value in message.channels)
        self.rc_received_s = self._now_s()

    def _pose(self, message: PoseStamped) -> None:
        self.pose = message
        self.pose_received_s = self._now_s()

    def _velocity(self, message: TwistStamped) -> None:
        self.velocity = message
        self.velocity_received_s = self._now_s()

    def _extended(self, message: ExtendedState) -> None:
        self.extended = message
        self.extended_received_s = self._now_s()

    def _estimator(self, message: EstimatorStatus) -> None:
        if self._fresh(self.ekf_report_received_s, self.ekf_maximum_age_s, self._now_s()):
            return
        self.estimator_healthy = bool(
            message.attitude_status_flag
            and message.velocity_horiz_status_flag
            and (message.pos_horiz_rel_status_flag or message.pos_horiz_abs_status_flag)
        )
        self.estimator_received_s = self._now_s()

    def _ekf_report(self, message: Mavlink) -> None:
        healthy = report_health(
            framing_status=int(message.framing_status),
            system_id=int(message.sysid),
            component_id=int(message.compid),
            message_id=int(message.msgid),
            length=int(message.len),
            payload64=message.payload64,
        )
        if healthy is None:
            return
        now = self._now_s()
        self.estimator_healthy = healthy
        self.estimator_received_s = now
        self.ekf_report_received_s = now
        self.ekf_report_count += 1

    def _home(self, _message: HomePosition) -> None:
        self.home_set = True
        self.home_received_s = self._now_s()

    def _candidate(self, message: PositionTarget) -> None:
        values = (message.velocity.x, message.velocity.y)
        if all(math.isfinite(value) for value in values):
            self.candidate = message
            self.candidate_received_s = self._now_s()

    def _target_status(self, message: String) -> None:
        now = self._now_s()
        self.target_status_received_s = now
        self.landing_alignment = None
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("IBVS status must be an object")
        except (TypeError, ValueError, json.JSONDecodeError):
            self.target_healthy = False
            self.target_aligned = False
            return
        self.target_healthy = payload.get("healthy") is True
        self.target_aligned = payload.get("aligned") is True
        if self.target_healthy:
            self.landing_alignment = LandingAlignment.from_payload(payload.get("landing_alignment"), now)

    def _landing_target_status(self, message: String) -> None:
        now = self._now_s()
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.landing_target_stream_healthy = False
            self.landing_target_output_enabled = False
            self.landing_target_status_received_s = now
            return
        self.landing_target_stream_healthy = payload.get("stream_healthy") is True
        self.landing_target_output_enabled = payload.get("output_enabled") is True
        self.landing_target_status_received_s = now

    def _range(self, message: Range) -> None:
        value = float(message.range)
        if (math.isfinite(value) and math.isfinite(message.min_range)
                and math.isfinite(message.max_range)
                and message.min_range <= value <= message.max_range):
            self.range_m = value
            self.range_received_s = self._now_s()

    def _rc_authorized(self, now: float) -> bool:
        if not self.require_rc:
            return True
        result = self.rc_gate.evaluate(
            self.rc_channels,
            received_time_s=self.rc_received_s,
            now_s=now,
        )
        healthy = bool(
            self._fresh(self.state_received_s, self.state_maximum_age_s, now)
            and self.vehicle_state.connected
            and self.vehicle_state.armed
        )
        entry_allowed = (
            self.vehicle_state.mode.strip().upper() in self.entry_modes
            and not self.lifecycle.active
        )
        return self.pilot_session.update(
            now_s=now,
            healthy=healthy,
            rc=result,
            received_s=self.rc_received_s,
            entry_allowed=entry_allowed,
        )

    def _landing_switch(self, now: float) -> tuple[bool, bool]:
        if self.rc_channels is None or self.rc_received_s is None:
            self.landing_switch_state = "MISSING"
            self.landing_switch_pwm = None
            return False, False
        index = self.landing_switch_channel - 1
        if index >= len(self.rc_channels):
            self.landing_switch_state = "MISSING"
            self.landing_switch_pwm = None
            return False, False
        age_s = now - self.rc_received_s
        pwm = int(self.rc_channels[index])
        self.landing_switch_pwm = pwm
        if (
            not math.isfinite(age_s)
            or not 0.0 <= age_s <= self.landing_switch_maximum_age_s
        ):
            self.landing_switch_state = "STALE"
            return False, False
        if pwm <= self.landing_switch_off_below_pwm:
            self.landing_switch_state = "LOW"
            return False, True
        if pwm >= self.landing_switch_on_above_pwm:
            self.landing_switch_state = "HIGH"
            return True, False
        self.landing_switch_state = "NEUTRAL"
        return False, False

    def _drive_rc_flight_flow(
        self,
        snapshot: VehicleSnapshot,
        now: float,
    ) -> None:
        if not self.rc_flight_flow_enabled:
            return
        active = self.lifecycle.request if self.lifecycle.active else None
        mode = snapshot.mode.strip().upper()
        if snapshot.landing_explicit_low:
            self.auto_land_inhibited = False
        can_start_land = bool(
            snapshot.authorized
            and snapshot.connected
            and snapshot.armed
            and snapshot.landing_requested
            and not self.auto_land_inhibited
            and not snapshot.landed
            and snapshot.range_fresh
            and math.isfinite(snapshot.range_m)
            and snapshot.range_m >= 0.0
            and snapshot.candidate_fresh
            and snapshot.landing_target_fresh
            and snapshot.landing_target_output_enabled
            and mode == self.guided_mode
        )
        guided_action_active = bool(active and active.kind in GUIDED_ACTIONS)
        if can_start_land and (
            guided_action_active or self.lifecycle.retaining_control
        ):
            if active is not None:
                self.lifecycle.cancel(
                    active.action_id,
                    now,
                    "CH8_LAND_PREEMPT",
                )
            self.auto_action_sequence += 1
            request = ActionRequest(
                action_id=f"rc-flow/land-{self.auto_action_sequence}",
                kind=ActionKind.LAND,
                timeout_s=3600.0,
                params={
                    "precision": True,
                    "rc_managed": True,
                    "loss_recovery": True,
                    "guided_descent": True,
                },
            )
            status = self.lifecycle.start(request, snapshot)
            self.auto_land_inhibited = True
            self._publish_status(status, now, event="RC_CH8_AUTO_LAND")
            return

        if active is not None:
            return
        can_start_follow = bool(
            snapshot.authorized
            and snapshot.connected
            and snapshot.armed
            and (not snapshot.landing_requested or self.auto_land_inhibited)
            and not snapshot.landed
            and snapshot.candidate_fresh
            and mode in self.entry_modes | {self.guided_mode}
        )
        if not can_start_follow:
            return
        # A rejected request (for example, unhealthy EKF) leaves no active
        # action. Limit retries so a held CH6 does not create 20 failures/s.
        if now - self.last_auto_follow_attempt_s < 1.0:
            return
        self.last_auto_follow_attempt_s = now
        self.auto_action_sequence += 1
        request = ActionRequest(
            action_id=f"rc-flow/follow-{self.auto_action_sequence}",
            kind=ActionKind.FOLLOW,
            timeout_s=3600.0,
            params={"maximum_speed_mps": self.lifecycle.maximum_horizontal_speed_mps},
        )
        status = self.lifecycle.start(request, snapshot)
        if (
            self.lifecycle.active
            and snapshot.landing_requested
            and self.auto_land_inhibited
        ):
            # LAND may exit while CH8 is still held high.  A subsequently
            # accepted FOLLOW proves that the Tag has been reacquired, so
            # re-arm CH8 without requiring a physical low -> high reset.  The
            # next GUIDED-confirmed tick will let LAND preempt FOLLOW again.
            self.auto_land_inhibited = False
        self._publish_status(status, now, event="RC_CH6_AUTO_GUIDED")

    def _snapshot(self, now: float) -> VehicleSnapshot:
        state_fresh = self._fresh(self.state_received_s, self.state_maximum_age_s, now)
        pose_fresh = self._fresh(self.pose_received_s, self.pose_maximum_age_s, now)
        velocity_fresh = self._fresh(self.velocity_received_s, self.velocity_maximum_age_s, now)
        estimator_fresh = self._fresh(self.estimator_received_s, self.ekf_maximum_age_s, now)
        home_fresh = self.home_set
        telemetry_fresh = bool(state_fresh and pose_fresh and velocity_fresh)
        position = (math.nan, math.nan, math.nan)
        yaw = math.nan
        tilt = math.nan
        if pose_fresh and self.pose is not None:
            p = self.pose.pose.position
            position = (float(p.x), float(p.y), float(p.z))
            q = self.pose.pose.orientation
            norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
            if math.isfinite(norm) and 0.9 <= norm <= 1.1:
                x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
                yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))
        velocity = (math.nan, math.nan, math.nan)
        yaw_rate = math.nan
        if velocity_fresh and self.velocity is not None:
            v = self.velocity.twist
            velocity = (float(v.linear.x), float(v.linear.y), float(v.linear.z))
            yaw_rate = float(v.angular.z)
        candidate_fresh = bool(
            self.candidate is not None
            and self.target_healthy
            and self._fresh(self.target_status_received_s, self.candidate_maximum_age_s, now)
            and self._fresh(self.candidate_received_s, self.candidate_maximum_age_s, now)
        )
        candidate = None
        if candidate_fresh and self.candidate is not None:
            candidate = (float(self.candidate.velocity.x), float(self.candidate.velocity.y))
        landing_target_status_fresh = self._fresh(
            self.landing_target_status_received_s,
            self.landing_target_maximum_age_s,
            now,
        )
        landed = bool(
            self.extended is not None
            and self._fresh(self.extended_received_s, self.state_maximum_age_s, now)
            and self.extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND
        )
        landing_requested, landing_explicit_low = self._landing_switch(now)
        return VehicleSnapshot(
            now_s=now,
            telemetry_fresh=telemetry_fresh,
            connected=bool(state_fresh and self.vehicle_state.connected),
            armed=bool(state_fresh and self.vehicle_state.armed),
            mode=self.vehicle_state.mode or "UNKNOWN",
            authorized=self._rc_authorized(now),
            ekf_healthy=bool(estimator_fresh and self.estimator_healthy),
            home_set=home_fresh,
            landed=landed,
            position_enu=position,
            velocity_enu=velocity,
            yaw_rad=yaw,
            yaw_rate_rad_s=yaw_rate,
            tilt_deg=tilt,
            candidate_velocity_flu=candidate,
            candidate_fresh=candidate_fresh,
            target_aligned=bool(candidate_fresh and self.target_aligned),
            landing_target_fresh=bool(
                landing_target_status_fresh
                and self.landing_target_stream_healthy
            ),
            landing_target_output_enabled=bool(
                landing_target_status_fresh
                and self.landing_target_output_enabled
            ),
            landing_requested=landing_requested,
            landing_explicit_low=landing_explicit_low,
            range_m=self.range_m,
            range_fresh=self._fresh(self.range_received_s, self.range_maximum_age_s, now),
            landing_alignment_fresh=bool(candidate_fresh and self.landing_alignment
                and self.landing_alignment.fresh(now, self.landing_alignment_maximum_age_s)),
            landing_velocity_flu=self.landing_alignment.velocity_flu if self.landing_alignment else None,
            landing_heading_error_rad=self.landing_alignment.heading_error_rad if self.landing_alignment else math.nan,
            landing_center_error_px=self.landing_alignment.center_error_px if self.landing_alignment else math.nan,
        )

    def _tick(self) -> None:
        now = self._now_s()
        recovering = self._peripheral_pre_tick(now)
        snapshot = self._snapshot(now)
        if not recovering:
            self._drive_rc_flight_flow(snapshot, now)
        snapshot = self._snapshot(now)
        status, command = self.lifecycle.tick(snapshot)
        if command is not None:
            self._execute(command, snapshot, now)
        self._update_tones(snapshot, status, now)
        self._publish_status(status, now, event="TICK")

    def _execute(self, command: ActionCommand, snapshot: VehicleSnapshot, now: float) -> None:
        if command.desired_mode and snapshot.mode.strip().upper() != command.desired_mode:
            self._request_mode(command.desired_mode, now)
        if command.velocity_enu is not None:
            target = self._position_target(command)
            self.preview_publisher.publish(target)
            if (self.output_enabled and snapshot.authorized and snapshot.telemetry_fresh
                    and snapshot.connected and snapshot.armed
                    and snapshot.mode.strip().upper() == self.guided_mode):
                self.setpoint_publisher.publish(target)
                self.last_sent_target = (tuple(command.velocity_enu), int(target.coordinate_frame), int(target.type_mask))
                self.last_sent_s = now
        if command.request_disarm:
            self._request_command("DISARM", now)
        if command.request_emergency_stop:
            self._request_command("EMERGENCY_STOP", now)

    def _request_mode(self, mode: str, now: float) -> None:
        if not self.output_enabled or not (
            self.lifecycle.active or self.lifecycle.retaining_control or self._orphan_rollback_due
        ):
            return
        action_id = (
            self.lifecycle.request.action_id
            if self.lifecycle.request is not None
            else "terminal-mode-command"
        )
        if self.mode_future is not None and not self.mode_future.done():
            return
        if (
            now - self.mode_request_s < self.mode_retry_s
            or not self.mode_client.service_is_ready()
        ):
            return
        request = SetMode.Request()
        request.custom_mode = mode
        normalized_mode = mode.strip().upper()
        self.mode_request_s = now
        self.mode_request_action_id = action_id
        self.owned_mode_deadlines[normalized_mode] = (
            now + self.mode_heartbeat_ack_timeout_s
        )
        self.mode_future = self.mode_client.call_async(request)
        self.mode_future.add_done_callback(
            lambda future, aid=action_id, requested_mode=normalized_mode:
                self._mode_result(future, aid, requested_mode)
        )

    def _mode_result(self, future, action_id: str, requested_mode: str) -> None:
        if not self.lifecycle.active or self.lifecycle.request.action_id != action_id:
            return
        try:
            sent = bool(future.result().mode_sent)
        except Exception as exc:
            self.owned_mode_deadlines.pop(requested_mode, None)
            self.get_logger().error(f"mode request transport failed: {exc}")
            return
        if not sent:
            self.owned_mode_deadlines.pop(requested_mode, None)
            self.lifecycle.fail(
                self._now_s(),
                "MODE_REQUEST_REJECTED",
                "WAITING_TASK_DECISION",
            )
        # A positive service reply is not completion.  _tick waits for State.mode.

    def _request_command(self, kind: str, now: float) -> None:
        enabled = self.allow_disarm if kind == "DISARM" else self.allow_emergency_stop
        if kind == "DISARM" and self.lifecycle.land_phase in {"NEAR_GROUND_DISARM", "LANDED_DISARM"}:
            enabled = self.allow_landing_disarm
        if not enabled or not self.lifecycle.active:
            return
        action_id = self.lifecycle.request.action_id
        if self.command_future is not None and not self.command_future.done():
            return
        if (
            now - self.command_request_s < self.command_retry_s
            or not self.command_client.service_is_ready()
        ):
            return
        request = CommandLong.Request()
        if kind == "DISARM":
            request.command = MAV_CMD_COMPONENT_ARM_DISARM
            request.param1 = 0.0
            request.param2 = 0.0
        else:
            request.command = MAV_CMD_DO_AUX_FUNCTION
            request.param1 = float(MOTOR_EMERGENCY_STOP_AUX_FUNCTION)
            request.param2 = 2.0
        self.command_request_s = now
        self.command_action_id = action_id
        self.command_kind = kind
        self.command_future = self.command_client.call_async(request)
        self.command_future.add_done_callback(
            lambda future, aid=action_id, command_kind=kind:
                self._command_result(future, aid, command_kind)
        )

    def _command_result(self, future, action_id: str, kind: str) -> None:
        if not self.lifecycle.active or self.lifecycle.request.action_id != action_id:
            return
        try:
            accepted = bool(future.result().success)
        except Exception as exc:
            if kind == "DISARM" and self.lifecycle.land_phase in {"NEAR_GROUND_DISARM", "LANDED_DISARM"}:
                self.lifecycle.reason = "DISARM_TRANSPORT_FAILED_CONTINUING_LAND"
                return
            self.lifecycle.fail(self._now_s(), f"{kind}_TRANSPORT_FAILED", str(exc))
            return
        if not accepted:
            if kind == "DISARM" and self.lifecycle.land_phase in {"NEAR_GROUND_DISARM", "LANDED_DISARM"}:
                self.lifecycle.reason = "DISARM_REJECTED_CONTINUING_LAND"
                return
            self.lifecycle.fail(self._now_s(), f"{kind}_REJECTED", "WAITING_TASK_DECISION")
        # DISARM completes only on armed=false.  E-stop has no trustworthy motor
        # feedback in the current graph and therefore stays RUNNING until timeout.

    def _position_target(self, command: ActionCommand) -> PositionTarget:
        target = PositionTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
        )
        velocity = command.velocity_enu or (0.0, 0.0, 0.0)
        target.velocity.x, target.velocity.y, target.velocity.z = velocity
        if command.yaw_rad is None:
            target.type_mask |= PositionTarget.IGNORE_YAW
        else:
            target.yaw = command.yaw_rad
        if command.yaw_rate_rad_s is None:
            target.type_mask |= PositionTarget.IGNORE_YAW_RATE
        else:
            target.yaw_rate = command.yaw_rate_rad_s
        return target

    def _publish_status(self, status, now: float, *, event: str) -> None:
        payload = status.as_dict()
        payload.update({
            "node": "ACTION_EXECUTOR_ROS2",
            "event": event,
            "environment": self.environment,
            "output_enabled": self.output_enabled,
            "mode_output_enabled": self.allow_mode,
            "setpoint_output_enabled": self.allow_setpoint,
            "disarm_output_enabled": self.allow_disarm,
            "landing_disarm_output_enabled": self.allow_landing_disarm,
            "emergency_stop_output_enabled": self.allow_emergency_stop,
            "pilot_session_authorized": self.pilot_session.enabled if self.require_rc else True,
            "pilot_session_sequence": self.pilot_session.session_sequence,
            "fcu_mode": self.vehicle_state.mode or "UNKNOWN",
            "land_phase": self.lifecycle.land_phase,
            "fcu_state_age_s": None if self.state_received_s is None else max(0.0, now - self.state_received_s),
            "pose_age_s": None if self.pose_received_s is None else max(0.0, now - self.pose_received_s),
            "velocity_age_s": None if self.velocity_received_s is None else max(0.0, now - self.velocity_received_s),
            "range_age_s": None if self.range_received_s is None else max(0.0, now - self.range_received_s),
            "range_m": self.range_m if math.isfinite(self.range_m) else None,
            "ekf_healthy": self.estimator_healthy,
            "ekf_report_count": self.ekf_report_count,
            "ekf_report_age_s": (
                None if self.ekf_report_received_s is None
                else max(0.0, now - self.ekf_report_received_s)
            ),
            "estimator_age_s": (
                None if self.estimator_received_s is None
                else max(0.0, now - self.estimator_received_s)
            ),
            "expected_mode": self._expected_mode(),
            "landing_target_stream_healthy": self.landing_target_stream_healthy,
            "landing_target_output_enabled": self.landing_target_output_enabled,
            "landing_alignment_fresh": bool(self.landing_alignment and
                self.landing_alignment.fresh(now, self.landing_alignment_maximum_age_s)),
            "landing_heading_error_deg": math.degrees(self.landing_alignment.heading_error_rad) if self.landing_alignment else None,
            "landing_center_error_px": self.landing_alignment.center_error_px if self.landing_alignment else None,
            "landing_adjust_while_descending": True,
            "landing_turn_while_descending": False,
            "landing_yaw_alignment_complete": self.lifecycle.land_yaw_alignment_complete,
            "rc_flight_flow_enabled": self.rc_flight_flow_enabled,
            "landing_switch_state": self.landing_switch_state,
            "landing_switch_pwm": self.landing_switch_pwm,
            "stamp_monotonic_s": now,
        })
        payload.update(self._peripheral_status(now))
        self._publish_event(payload)

    def _publish_event(self, payload: dict) -> None:
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        self.status_publisher.publish(message)

    def _expected_mode(self) -> Optional[str]:
        if self._orphan_rollback_due:
            return self.lifecycle.fallback_mode
        return self.lifecycle.expected_mode

    @staticmethod
    def _fresh(received_s: Optional[float], maximum_age_s: float, now: float) -> bool:
        return bool(
            received_s is not None
            and math.isfinite(received_s)
            and 0.0 <= now - received_s <= maximum_age_s
        )

    @staticmethod
    def _acquire_single_writer_lock(path: str):
        raw_path = path.strip()
        if not raw_path:
            raise ValueError("single_writer_lock_path is required")
        path = os.path.abspath(os.path.expanduser(raw_path))
        handle = open(path, "a+b")
        try:
            if os.name == "nt":
                import msvcrt
                if os.path.getsize(path) == 0:
                    handle.write(b"0")
                    handle.flush()
                handle.seek(0)
                msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (IOError, OSError) as exc:
            handle.close()
            raise RuntimeError("another action executor owns the MAVROS writer lock") from exc
        return handle


def main() -> None:
    rclpy.init()
    node = ActionExecutorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
