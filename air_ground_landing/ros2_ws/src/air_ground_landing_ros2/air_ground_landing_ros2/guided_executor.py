"""Single-writer ROS 2 executor for ArduPilot GUIDED setpoints."""

from __future__ import annotations

import json
import math
import time
from copy import deepcopy
from typing import Optional

import rclpy
from mavros_msgs.msg import Mavlink, PositionTarget, RCIn, State
from mavros_msgs.srv import CommandLong, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String

from air_ground_landing.guided_execution import (
    ModeRequest,
    ModeTransitionConfig,
    ModeTransitionManager,
    ModeTransitionPhase,
    HorizontalVelocityLimitConfig,
    HorizontalVelocityLimiter,
    RcAuthorizationGate,
    RcGateConfig,
    RcGateResult,
    RcGateState,
    RcLandingRequestGate,
    LandingSwitchConfig,
    LandingSwitchResult,
)
from air_ground_landing.follow_tone_policy import (
    FollowToneEvent,
    FollowTonePolicy,
    TUNES,
)
from air_ground_landing.legacy_mavlink_tune import (
    MAVLINK_V2_MAGIC,
    PLAY_TUNE_MSG_ID,
    encode_legacy_play_tune,
)


GUIDED_OWNERS = {"ELASTIC_GUIDED", "IBVS_GUIDED"}
LAND_OWNER = "AC_PRECLAND_LAND"
MAV_CMD_SET_MESSAGE_INTERVAL = 511
POSITION_TARGET_LOCAL_NED_MESSAGE_ID = 85


class GuidedExecutor(Node):
    """Forward exactly one fresh candidate after RC and heartbeat confirmation."""

    def __init__(self) -> None:
        super().__init__("guided_executor")
        self._declare_parameters()
        environment = str(self.get_parameter("environment").value).lower()
        if environment not in ("offline", "sitl", "hardware"):
            raise ValueError("environment must be offline, sitl or hardware")
        flight_approved = bool(self.get_parameter("flight_use_approved").value)
        self.require_rc = bool(self.get_parameter("require_rc_authorization").value)
        if environment == "hardware" and not self.require_rc:
            raise ValueError("hardware execution requires the RC authorization gate")
        environment_approved = environment == "sitl" or (
            environment == "hardware" and flight_approved
        )
        self.mode_output_enabled = bool(
            self.get_parameter("allow_mode_change").value
        ) and environment_approved
        self.setpoint_output_enabled = bool(
            self.get_parameter("allow_setpoint_output").value
        ) and environment_approved
        # A partial enable is fail-closed: neither output path is active.
        self.execution_enabled = self.mode_output_enabled and self.setpoint_output_enabled

        self.candidate_timeout_s = float(self.get_parameter("candidate_timeout_s").value)
        self.owner_timeout_s = float(self.get_parameter("owner_timeout_s").value)
        self.target_echo_timeout_s = float(
            self.get_parameter("target_echo_timeout_s").value
        )
        if (
            self.candidate_timeout_s <= 0.0
            or self.owner_timeout_s <= 0.0
            or self.target_echo_timeout_s <= 0.0
        ):
            raise ValueError("GUIDED candidate/owner timeouts must be positive")
        self.manager = ModeTransitionManager(
            ModeTransitionConfig(
                target_ack_timeout_s=float(
                    self.get_parameter("target_heartbeat_ack_timeout_s").value
                ),
                rollback_ack_timeout_s=float(
                    self.get_parameter("rollback_heartbeat_ack_timeout_s").value
                ),
                rollback_retry_interval_s=float(
                    self.get_parameter("rollback_retry_interval_s").value
                ),
                fallback_mode=str(self.get_parameter("rollback_mode").value),
            )
        )
        self.horizontal_limiter = HorizontalVelocityLimiter(
            HorizontalVelocityLimitConfig(
                maximum_speed_mps=float(
                    self.get_parameter("maximum_horizontal_speed_mps").value
                ),
                maximum_acceleration_mps2=float(
                    self.get_parameter("maximum_horizontal_acceleration_mps2").value
                ),
            )
        )
        self.guided_mode = str(self.get_parameter("target_mode").value).strip().upper()
        self.land_mode = str(self.get_parameter("land_mode").value).strip().upper()
        self.ready_entry_modes = {
            str(value).strip().upper()
            for value in self.get_parameter("ready_entry_modes").value
        }
        if not self.guided_mode or not self.land_mode:
            raise ValueError("GUIDED and LAND mode names are required")
        if not self.ready_entry_modes:
            raise ValueError("ready_entry_modes must not be empty")
        self.rc_gate = RcAuthorizationGate(
            RcGateConfig(
                channel=int(self.get_parameter("rc_channel").value),
                abort_below_pwm=int(self.get_parameter("rc_abort_below_pwm").value),
                authorize_above_pwm=int(
                    self.get_parameter("rc_authorize_above_pwm").value
                ),
                maximum_age_s=float(self.get_parameter("rc_maximum_age_s").value),
            )
        )
        self.landing_switch = RcLandingRequestGate(
            LandingSwitchConfig(
                channel=int(self.get_parameter("landing_switch_channel").value),
                off_below_pwm=int(
                    self.get_parameter("landing_switch_off_below_pwm").value
                ),
                on_above_pwm=int(
                    self.get_parameter("landing_switch_on_above_pwm").value
                ),
                maximum_age_s=float(
                    self.get_parameter("landing_switch_maximum_age_s").value
                ),
            )
        )

        self.vehicle_state = State()
        self.vehicle_state.connected = False
        self.owner = "NONE"
        self.owner_received_s: Optional[float] = None
        self.rc_channels: Optional[tuple[int, ...]] = None
        self.rc_received_s: Optional[float] = None
        self.candidates: dict[str, tuple[PositionTarget, float]] = {}
        self.target_echo: Optional[PositionTarget] = None
        self.target_echo_received_s: Optional[float] = None
        self.target_echo_received_wall_s: Optional[float] = None
        self.target_echo_continuous_since_s: Optional[float] = None
        self.target_echo_streak_count = 0
        self.target_echo_received_count = 0
        self.last_setpoint_sent_s: Optional[float] = None
        self.last_setpoint_sent_wall_s: Optional[float] = None
        self.last_setpoint_sent: Optional[PositionTarget] = None
        self.connected_since_s: Optional[float] = None
        self.target_echo_interval_pending = False
        self.target_echo_interval_confirmed = False
        self.target_echo_interval_result: Optional[int] = None
        self.next_target_echo_interval_request_s = 0.0
        self.tag_detected_received_s: Optional[float] = None
        self.tag_detection_timeout_s = float(
            self.get_parameter("tag_detection_timeout_s").value
        )
        self.follow_active_tone_repeat_s = float(
            self.get_parameter("follow_active_tone_repeat_s").value
        )
        self.landing_active_tone_repeat_s = float(
            self.get_parameter("landing_active_tone_repeat_s").value
        )
        if self.tag_detection_timeout_s <= 0.0:
            raise ValueError("tag_detection_timeout_s must be positive")
        if self.follow_active_tone_repeat_s <= 0.0:
            raise ValueError("follow_active_tone_repeat_s must be positive")
        if self.landing_active_tone_repeat_s <= 0.0:
            raise ValueError("landing_active_tone_repeat_s must be positive")
        self.tone_output_enabled = bool(
            self.get_parameter("tone_output_enabled").value
        )
        self.tone_source_system = int(self.get_parameter("tone_source_system").value)
        self.tone_source_component = int(
            self.get_parameter("tone_source_component").value
        )
        self.tone_target_system = int(self.get_parameter("tone_target_system").value)
        self.tone_target_component = int(
            self.get_parameter("tone_target_component").value
        )
        self.tone_sequence = 0
        self.tone_policy = FollowTonePolicy(
            follow_repeat_interval_s=self.follow_active_tone_repeat_s,
            landing_repeat_interval_s=self.landing_active_tone_repeat_s,
        )
        self.tone_event_counts = {event.value: 0 for event in FollowToneEvent}

        self.actual_publisher = self.create_publisher(
            PositionTarget,
            str(self.get_parameter("mavros_output_topic").value),
            10,
        )
        self.preview_publisher = self.create_publisher(
            PositionTarget,
            str(self.get_parameter("preview_topic").value),
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.tone_publisher = self.create_publisher(
            Mavlink,
            str(self.get_parameter("tone_mavlink_sink_topic").value),
            qos_profile_sensor_data,
        )
        self.landing_request_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("landing_request_topic").value),
            10,
        )
        self.mode_client = self.create_client(
            SetMode,
            str(self.get_parameter("mavros_set_mode_service").value),
        )
        self.command_client = self.create_client(
            CommandLong,
            str(self.get_parameter("mavros_command_service").value),
        )
        self.create_subscription(
            PositionTarget,
            str(self.get_parameter("elastic_candidate_topic").value),
            lambda message: self._candidate("ELASTIC_GUIDED", message),
            10,
        )
        self.create_subscription(
            PositionTarget,
            str(self.get_parameter("ibvs_candidate_topic").value),
            lambda message: self._candidate("IBVS_GUIDED", message),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("control_owner_topic").value),
            self._control_owner,
            10,
        )
        self.create_subscription(
            State,
            str(self.get_parameter("mavros_state_topic").value),
            self._vehicle_state,
            10,
        )
        self.create_subscription(
            RCIn,
            str(self.get_parameter("mavros_rc_topic").value),
            self._rc,
            10,
        )
        self.create_subscription(
            PositionTarget,
            str(self.get_parameter("target_echo_topic").value),
            self._target_echo,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("tag_detection_status_topic").value),
            self._tag_detection_status,
            qos_profile_sensor_data,
        )
        self.create_timer(0.05, self._tick)

    def _declare_parameters(self) -> None:
        defaults = {
            "environment": "offline",
            "flight_use_approved": False,
            "allow_mode_change": False,
            "allow_setpoint_output": False,
            "require_rc_authorization": True,
            "rc_channel": 6,
            "rc_abort_below_pwm": 1300,
            "rc_authorize_above_pwm": 1800,
            "rc_maximum_age_s": 0.5,
            "landing_switch_channel": 8,
            "landing_switch_off_below_pwm": 1200,
            "landing_switch_on_above_pwm": 1800,
            "landing_switch_maximum_age_s": 0.5,
            "landing_request_topic": "/landing/descent_request",
            "candidate_timeout_s": 0.4,
            "owner_timeout_s": 0.5,
            "target_echo_timeout_s": 0.5,
            "target_mode": "GUIDED",
            "land_mode": "LAND",
            "rollback_mode": "LOITER",
            "ready_entry_modes": ["ALT_HOLD", "LOITER", "POSHOLD", "GUIDED"],
            "target_heartbeat_ack_timeout_s": 2.0,
            "rollback_heartbeat_ack_timeout_s": 2.0,
            "rollback_retry_interval_s": 1.0,
            "rollback_orphaned_guided": True,
            "orphaned_guided_grace_s": 1.0,
            "maximum_horizontal_speed_mps": 0.10,
            "maximum_horizontal_acceleration_mps2": 0.15,
            "elastic_candidate_topic": "/landing/elastic/candidate",
            "ibvs_candidate_topic": "/landing/ibvs/candidate",
            "control_owner_topic": "/landing/control_owner",
            "preview_topic": "/landing/guided_executor/preview",
            "status_topic": "/landing/guided_executor/status",
            "tone_output_enabled": False,
            "tone_mavlink_sink_topic": "/uas1/mavlink_sink",
            "tone_source_system": 191,
            "tone_source_component": 191,
            "tone_target_system": 1,
            "tone_target_component": 1,
            "tag_detection_status_topic": "/landing/landing_target/status",
            "tag_detection_timeout_s": 0.5,
            "follow_active_tone_repeat_s": 3.0,
            "landing_active_tone_repeat_s": 2.0,
            "target_echo_topic": "/mavros/setpoint_raw/target_local",
            "mavros_output_topic": "/mavros/setpoint_raw/local",
            "mavros_state_topic": "/mavros/state",
            "mavros_rc_topic": "/mavros/rc/in",
            "mavros_set_mode_service": "/mavros/set_mode",
            "mavros_command_service": "/mavros/cmd/command",
            "target_echo_message_id": POSITION_TARGET_LOCAL_NED_MESSAGE_ID,
            "target_echo_interval_us": 200000.0,
            "target_echo_request_retry_s": 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    @staticmethod
    def _now_s() -> float:
        return time.monotonic()

    def _candidate(self, owner: str, message: PositionTarget) -> None:
        self.candidates[owner] = (message, self._now_s())

    def _control_owner(self, message: String) -> None:
        raw = message.data.strip()
        try:
            decoded = json.loads(raw)
            owner = decoded.get("control_owner", decoded.get("owner", "NONE"))
        except (json.JSONDecodeError, AttributeError):
            owner = raw
        self.owner = str(owner).strip().upper()
        self.owner_received_s = self._now_s()

    def _vehicle_state(self, message: State) -> None:
        was_connected = bool(self.vehicle_state.connected)
        self.vehicle_state = message
        if message.connected and not was_connected:
            self.connected_since_s = self._now_s()
            self.target_echo_interval_pending = False
            self.target_echo_interval_confirmed = False
            self.target_echo_interval_result = None
            self.next_target_echo_interval_request_s = 0.0
        elif not message.connected:
            self.connected_since_s = None
            self.target_echo_interval_pending = False
            self.target_echo_interval_confirmed = False

    def _rc(self, message: RCIn) -> None:
        self.rc_channels = tuple(int(value) for value in message.channels)
        self.rc_received_s = self._now_s()

    def _target_echo(self, message: PositionTarget) -> None:
        now_s = self._now_s()
        if (
            self.target_echo_received_s is None
            or now_s - self.target_echo_received_s > self.target_echo_timeout_s
        ):
            self.target_echo_continuous_since_s = now_s
            self.target_echo_streak_count = 1
        else:
            self.target_echo_streak_count += 1
        self.target_echo = message
        self.target_echo_received_s = now_s
        self.target_echo_received_wall_s = time.time()
        self.target_echo_received_count += 1

    def _tag_detection_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if bool(status.get("accepted_this_poll", False)):
            self.tag_detected_received_s = self._now_s()

    def _tag_detected(self, now_s: float) -> bool:
        return bool(
            self.tag_detected_received_s is not None
            and now_s - self.tag_detected_received_s <= self.tag_detection_timeout_s
        )

    def _rc_result(self, now_s: float) -> RcGateResult:
        if not self.require_rc:
            return RcGateResult(RcGateState.AUTHORIZED, None, 0.0)
        return self.rc_gate.evaluate(
            self.rc_channels,
            received_time_s=self.rc_received_s,
            now_s=now_s,
        )

    def _authorized_candidate(self, now_s: float, rc: RcGateResult):
        if not self.vehicle_state.connected:
            return None, "MAVROS_DISCONNECTED"
        if self.owner_received_s is None or now_s - self.owner_received_s > self.owner_timeout_s:
            return None, "CONTROL_OWNER_STALE"
        if self.owner not in GUIDED_OWNERS:
            return None, f"OWNER_{self.owner}_NOT_GUIDED"
        if not rc.authorized:
            return None, f"RC_{rc.state.value}"
        entry = self.candidates.get(self.owner)
        if entry is None:
            return None, "CANDIDATE_MISSING"
        candidate, received_s = entry
        if now_s - received_s > self.candidate_timeout_s:
            return None, "CANDIDATE_STALE"
        return candidate, "CANDIDATE_AUTHORIZED"

    def _ready_candidate_without_rc(self, now_s: float):
        if not self.vehicle_state.connected:
            return None, "MAVROS_DISCONNECTED"
        if self.owner_received_s is None or now_s - self.owner_received_s > self.owner_timeout_s:
            return None, "CONTROL_OWNER_STALE"
        if self.owner not in GUIDED_OWNERS:
            return None, f"OWNER_{self.owner}_NOT_GUIDED"
        mode = (self.vehicle_state.mode or "UNKNOWN").upper()
        if mode not in self.ready_entry_modes:
            return None, f"MODE_{mode}_NOT_READY"
        entry = self.candidates.get(self.owner)
        if entry is None:
            return None, "CANDIDATE_MISSING"
        candidate, received_s = entry
        if now_s - received_s > self.candidate_timeout_s:
            return None, "CANDIDATE_STALE"
        return candidate, "OBSERVE_READY"

    def _target_echo_fresh(self, now_s: float) -> bool:
        return bool(
            self.target_echo is not None
            and self.target_echo_received_s is not None
            and now_s - self.target_echo_received_s <= self.target_echo_timeout_s
        )

    def _emit_tones(self, events: tuple[FollowToneEvent, ...]) -> None:
        for event in events:
            self.tone_event_counts[event.value] += 1
            if self.tone_output_enabled:
                frame = encode_legacy_play_tune(
                    TUNES[event],
                    sequence=self.tone_sequence,
                    source_system=self.tone_source_system,
                    source_component=self.tone_source_component,
                    target_system=self.tone_target_system,
                    target_component=self.tone_target_component,
                )
                self.tone_sequence = (self.tone_sequence + 1) & 0xFF
                message = Mavlink()
                message.header.stamp = self.get_clock().now().to_msg()
                message.framing_status = Mavlink.FRAMING_OK
                message.magic = MAVLINK_V2_MAGIC
                message.len = frame.payload_length
                message.incompat_flags = 0
                message.compat_flags = 0
                message.seq = frame.sequence
                message.sysid = frame.source_system
                message.compid = frame.source_component
                message.msgid = PLAY_TUNE_MSG_ID
                message.checksum = frame.checksum
                message.payload64 = list(frame.payload64)
                message.signature = []
                self.tone_publisher.publish(message)
            self.get_logger().info(
                "FOLLOW_TONE_EVENT "
                + json.dumps(
                    {
                        "event": event.value,
                        "transmitted": self.tone_output_enabled,
                        "tune": TUNES[event],
                        "transport": "MAVLINK_PLAY_TUNE_LEGACY",
                        "mavlink_msgid": PLAY_TUNE_MSG_ID,
                    },
                    separators=(",", ":"),
                )
            )

    def _owner_fresh(self, now_s: float) -> bool:
        return bool(
            self.owner_received_s is not None
            and now_s - self.owner_received_s <= self.owner_timeout_s
        )

    def _follow_session_active(self, now_s: float, rc: RcGateResult) -> bool:
        if not self.vehicle_state.connected or not rc.authorized or not self._owner_fresh(now_s):
            return False
        mode = (self.vehicle_state.mode or "UNKNOWN").upper()
        transition = self.manager.status()
        confirmed_guided_follow = bool(
            self.owner in GUIDED_OWNERS
            and mode == self.guided_mode
            and transition.target_mode == self.guided_mode
            and transition.setpoint_stream_authorized
        )
        continuing_landing_session = bool(
            self.landing_switch.requested
            and self.owner == LAND_OWNER
            and mode in {self.guided_mode, self.land_mode}
        )
        return confirmed_guided_follow or continuing_landing_session

    def _desired_mode(
        self,
        *,
        now_s: float,
        rc: RcGateResult,
        landing: LandingSwitchResult,
        guided_candidate,
    ) -> tuple[Optional[str], str]:
        if not self.execution_enabled:
            return None, "EXECUTION_DISABLED"
        transition = self.manager.status()
        connected_age_s = (
            None
            if self.connected_since_s is None
            else max(0.0, now_s - self.connected_since_s)
        )
        if (
            bool(self.get_parameter("rollback_orphaned_guided").value)
            and transition.phase == ModeTransitionPhase.IDLE
            and self.vehicle_state.connected
            and (self.vehicle_state.mode or "UNKNOWN").upper() == self.guided_mode
            and guided_candidate is None
            and connected_age_s is not None
            and connected_age_s
            >= float(self.get_parameter("orphaned_guided_grace_s").value)
        ):
            return str(self.get_parameter("rollback_mode").value).upper(), "ORPHANED_GUIDED_ROLLBACK"
        if self.owner == LAND_OWNER:
            if not self.vehicle_state.connected:
                return None, "MAVROS_DISCONNECTED"
            if not self._owner_fresh(now_s):
                return None, "CONTROL_OWNER_STALE"
            if not rc.authorized:
                return None, f"FOLLOW_RC_{rc.state.value}"
            if landing.requested:
                return self.land_mode, "SWD_LAND_AUTHORIZED"
            # SwD OFF means resume the already-authorized follow session.  Ask
            # for GUIDED immediately; fresh setpoints remain owner-gated.
            return self.guided_mode, f"SWD_{landing.state.value}_RESUME_GUIDED"
        if guided_candidate is not None:
            return self.guided_mode, "GUIDED_CANDIDATE_AUTHORIZED"
        return None, "NO_AUTHORIZED_MODE_REQUEST"

    def _tick(self) -> None:
        now_s = self._now_s()
        self._ensure_target_echo_interval(now_s)
        rc = self._rc_result(now_s)
        candidate, gate_reason = self._authorized_candidate(now_s, rc)
        ready_candidate, readiness_reason = self._ready_candidate_without_rc(now_s)
        follow_session_active = self._follow_session_active(now_s, rc)
        landing = self.landing_switch.evaluate(
            self.rc_channels,
            received_time_s=self.rc_received_s,
            now_s=now_s,
            follow_active=follow_session_active,
        )
        landing_message = Bool()
        landing_message.data = landing.requested
        self.landing_request_publisher.publish(landing_message)
        desired_mode, mode_gate_reason = self._desired_mode(
            now_s=now_s,
            rc=rc,
            landing=landing,
            guided_candidate=candidate,
        )
        request = self.manager.update(
            now_s=now_s,
            current_mode=self.vehicle_state.mode or "UNKNOWN",
            desired_mode=desired_mode,
        )
        if request is not None:
            self._dispatch_mode_request(request)
        transition = self.manager.status()
        output_candidate = None
        output_authorized = bool(
            candidate is not None
            and self.execution_enabled
            and transition.setpoint_stream_authorized
            and transition.target_mode == self.guided_mode
        )
        if output_authorized:
            output_candidate = deepcopy(candidate)
            limited_vx, limited_vy = self.horizontal_limiter.apply(
                output_candidate.velocity.x,
                output_candidate.velocity.y,
                now_s=now_s,
            )
            output_candidate.velocity.x = limited_vx
            output_candidate.velocity.y = limited_vy
            output_candidate.header.stamp = self.get_clock().now().to_msg()
            self.preview_publisher.publish(output_candidate)
            self.actual_publisher.publish(output_candidate)
            self.last_setpoint_sent = deepcopy(output_candidate)
            self.last_setpoint_sent_s = now_s
            self.last_setpoint_sent_wall_s = time.time()
        else:
            self.horizontal_limiter.reset()
            if candidate is not None:
                preview_candidate = deepcopy(candidate)
                preview_candidate.header.stamp = self.get_clock().now().to_msg()
                self.preview_publisher.publish(preview_candidate)
        control_active = bool(
            output_candidate is not None
            and self.execution_enabled
            and transition.setpoint_stream_authorized
            and transition.target_mode == self.guided_mode
            and (self.vehicle_state.mode or "UNKNOWN").upper() == self.guided_mode
        )
        target_echo_fresh = self._target_echo_fresh(now_s)
        follow_active = bool(control_active and target_echo_fresh)
        landing_active = bool(
            self.vehicle_state.connected
            and self.vehicle_state.armed
            and self._owner_fresh(now_s)
            and self.owner == LAND_OWNER
            and landing.requested
            and (self.vehicle_state.mode or "UNKNOWN").upper() == self.land_mode
        )
        tag_detected = self._tag_detected(now_s)
        # Do not consume the one-shot Tag event while MAVROS is unable to
        # deliver PLAY_TUNE.  Follow tones additionally require a fresh ID 85
        # target echo, but no longer compare echoed and commanded velocities.
        tag_tone_ready = bool(tag_detected and self.vehicle_state.connected)
        current_mode = (self.vehicle_state.mode or "UNKNOWN").upper()
        exit_confirmed = bool(
            current_mode not in {self.guided_mode, self.land_mode}
            or (current_mode == self.land_mode and not self.vehicle_state.armed)
        )
        tone_events = self.tone_policy.update(
            observe_ready=tag_tone_ready,
            follow_active=follow_active,
            landing_active=landing_active,
            exit_confirmed=exit_confirmed,
            now_s=now_s,
        )
        self._emit_tones(tone_events)
        self._publish_status(
            rc,
            landing,
            gate_reason,
            mode_gate_reason,
            follow_session_active,
            readiness_reason,
            ready_candidate is not None,
            tag_detected,
            target_echo_fresh,
            follow_active,
            landing_active,
            tone_events,
        )

    def _ensure_target_echo_interval(self, now_s: float) -> None:
        if (
            not self.vehicle_state.connected
            or self.target_echo_interval_confirmed
            or self.target_echo_interval_pending
            or now_s < self.next_target_echo_interval_request_s
            or not self.command_client.service_is_ready()
        ):
            return
        request = CommandLong.Request()
        request.broadcast = False
        request.command = MAV_CMD_SET_MESSAGE_INTERVAL
        request.confirmation = 0
        request.param1 = float(self.get_parameter("target_echo_message_id").value)
        request.param2 = float(self.get_parameter("target_echo_interval_us").value)
        request.param3 = 0.0
        request.param4 = 0.0
        request.param5 = 0.0
        request.param6 = 0.0
        request.param7 = 0.0
        self.target_echo_interval_pending = True
        future = self.command_client.call_async(request)

        def completed(result_future) -> None:
            self.target_echo_interval_pending = False
            try:
                response = result_future.result()
                self.target_echo_interval_result = int(response.result)
                self.target_echo_interval_confirmed = bool(response.success)
            except Exception:
                self.target_echo_interval_result = None
                self.target_echo_interval_confirmed = False
            if not self.target_echo_interval_confirmed:
                self.next_target_echo_interval_request_s = self._now_s() + float(
                    self.get_parameter("target_echo_request_retry_s").value
                )

        future.add_done_callback(completed)

    def _dispatch_mode_request(self, action: ModeRequest) -> None:
        if not self.execution_enabled or not self.mode_client.service_is_ready():
            followup = self.manager.on_service_result(
                sequence=action.sequence,
                mode_sent=False,
                now_s=self._now_s(),
            )
            if followup is not None and followup.sequence != action.sequence:
                self._dispatch_mode_request(followup)
            return
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = action.mode
        future = self.mode_client.call_async(request)

        def completed(result_future) -> None:
            try:
                mode_sent = bool(result_future.result().mode_sent)
            except Exception:
                mode_sent = False
            followup = self.manager.on_service_result(
                sequence=action.sequence,
                mode_sent=mode_sent,
                now_s=self._now_s(),
            )
            if followup is not None:
                self._dispatch_mode_request(followup)

        future.add_done_callback(completed)

    def _publish_status(
        self,
        rc: RcGateResult,
        landing: LandingSwitchResult,
        gate_reason: str,
        mode_gate_reason: str,
        follow_session_active: bool,
        readiness_reason: str,
        observe_ready: bool,
        tag_detected: bool,
        target_echo_fresh: bool,
        follow_active: bool,
        landing_active: bool,
        tone_events: tuple[FollowToneEvent, ...],
    ) -> None:
        status = self.manager.status().as_dict()
        transition = self.manager.status()
        status.update(
            {
                "node": "GUIDED_EXECUTOR_ROS2",
                "execution_enabled": self.execution_enabled,
                "setpoint_transmitted": bool(
                    self.execution_enabled
                    and transition.setpoint_stream_authorized
                    and transition.target_mode == self.guided_mode
                ),
                "control_owner": self.owner,
                "candidate_gate": gate_reason,
                "mode_gate": mode_gate_reason,
                "follow_session_active": follow_session_active,
                "observe_ready": observe_ready,
                "readiness_reason": readiness_reason,
                "tag_detected": tag_detected,
                "tag_detection_age_s": None
                if self.tag_detected_received_s is None
                else max(0.0, self._now_s() - self.tag_detected_received_s),
                "follow_active": follow_active,
                "landing_active": landing_active,
                "follow_active_tone_repeat_s": self.follow_active_tone_repeat_s,
                "landing_active_tone_repeat_s": self.landing_active_tone_repeat_s,
                "target_echo_fresh": target_echo_fresh,
                "target_echo_interval_requested": self.target_echo_interval_confirmed,
                "target_echo_interval_result": self.target_echo_interval_result,
                "target_echo_message_id": int(
                    self.get_parameter("target_echo_message_id").value
                ),
                "target_echo_interval_us": float(
                    self.get_parameter("target_echo_interval_us").value
                ),
                "target_echo_age_s": None
                if self.target_echo_received_s is None
                else max(0.0, self._now_s() - self.target_echo_received_s),
                "target_echo_received_unix_s": self.target_echo_received_wall_s,
                "target_echo_received_count": self.target_echo_received_count,
                "target_echo_streak_count": self.target_echo_streak_count,
                "target_echo_continuous": bool(
                    target_echo_fresh and self.target_echo_streak_count >= 2
                ),
                "target_echo_continuous_duration_s": None
                if not target_echo_fresh or self.target_echo_continuous_since_s is None
                else max(0.0, self._now_s() - self.target_echo_continuous_since_s),
                "target_echo_velocity_mps": self._velocity_dict(self.target_echo),
                "latest_sent_velocity_mps": self._velocity_dict(self.last_setpoint_sent),
                "latest_sent_unix_s": self.last_setpoint_sent_wall_s,
                "echo_minus_latest_sent_velocity_mps": self._velocity_difference(),
                "tone_output_enabled": self.tone_output_enabled,
                "tone_transport": "MAVLINK_PLAY_TUNE_LEGACY",
                "tone_events": [event.value for event in tone_events],
                "tone_event_counts": dict(self.tone_event_counts),
                "follow_rc_gate": rc.state.value,
                "follow_rc_pwm": rc.pwm,
                "landing_switch_state": landing.state.value,
                "landing_switch_pwm": landing.pwm,
                "landing_requested": landing.requested,
            }
        )
        message = String()
        message.data = json.dumps(status, separators=(",", ":"))
        self.status_publisher.publish(message)

    @staticmethod
    def _velocity_dict(target: Optional[PositionTarget]) -> Optional[dict[str, float]]:
        if target is None:
            return None
        return {
            "x": float(target.velocity.x),
            "y": float(target.velocity.y),
            "z": float(target.velocity.z),
        }

    def _velocity_difference(self) -> Optional[dict[str, float]]:
        if self.target_echo is None or self.last_setpoint_sent is None:
            return None
        difference = {
            axis: echo - sent
            for axis, echo, sent in (
                ("x", float(self.target_echo.velocity.x), float(self.last_setpoint_sent.velocity.x)),
                ("y", float(self.target_echo.velocity.y), float(self.last_setpoint_sent.velocity.y)),
                ("z", float(self.target_echo.velocity.z), float(self.last_setpoint_sent.velocity.z)),
            )
        }
        difference["horizontal_norm"] = math.hypot(difference["x"], difference["y"])
        return difference


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GuidedExecutor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
