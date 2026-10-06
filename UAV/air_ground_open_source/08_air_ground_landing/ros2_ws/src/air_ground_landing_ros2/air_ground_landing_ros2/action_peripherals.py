"""Peripheral services owned by the single action executor (no second controller)."""
import math

from mavros_msgs.msg import Mavlink, PositionTarget
from mavros_msgs.srv import CommandLong
from rclpy.qos import qos_profile_sensor_data

from air_ground_landing.action_execution import ActionKind
from air_ground_landing.executor_peripherals import StartupGuidedRecovery, TargetEchoMonitor
from air_ground_landing.follow_tone_policy import FollowTonePolicy, FollowToneEvent, TUNES
from air_ground_landing.legacy_mavlink_tune import (
    encode_legacy_play_tune, MAVLINK_V2_MAGIC, PLAY_TUNE_MSG_ID,
)


PERIPHERAL_DEFAULTS = {
    "tone_output_enabled": False,
    "tone_mavlink_sink_topic": "/uas1/mavlink_sink",
    "tone_source_system": 191,
    "tone_source_component": 191,
    "tone_target_system": 1,
    "tone_target_component": 1,
    "follow_active_tone_repeat_s": 3.0,
    "landing_active_tone_repeat_s": 2.0,
    "allow_target_echo_request": False,
    "target_echo_topic": "/mavros/setpoint_raw/target_local",
    "target_echo_timeout_s": .5,
    "target_echo_message_id": 85,
    "target_echo_interval_us": 200000.0,
    "target_echo_request_retry_s": 2.0,
    "rollback_orphaned_guided": True,
    "orphaned_guided_grace_s": 1.0,
}


class ActionPeripherals:
    def _init_peripherals(self, approved):
        param = lambda name: self.get_parameter(name).value
        for name in ("target_echo_timeout_s", "target_echo_interval_us",
                     "target_echo_request_retry_s", "orphaned_guided_grace_s",
                     "follow_active_tone_repeat_s", "landing_active_tone_repeat_s"):
            value = float(param(name))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if int(param("target_echo_message_id")) != 85:
            raise ValueError("target echo must request POSITION_TARGET_LOCAL_NED (85)")
        for name in ("tone_source_system", "tone_source_component", "tone_target_system", "tone_target_component"):
            if not 1 <= int(param(name)) <= 255:
                raise ValueError(f"{name} must be in [1,255]")
        self.tone_output_enabled = approved and bool(param("tone_output_enabled"))
        self.allow_target_echo_request = approved and bool(param("allow_target_echo_request"))
        self.echo_monitor = TargetEchoMonitor(float(param("target_echo_timeout_s")))
        self.orphan_recovery = StartupGuidedRecovery(float(param("orphaned_guided_grace_s")), self.guided_mode)
        self._orphan_rollback_due = False
        self.echo_link_connected = False
        self.echo_request_epoch = 0
        self.echo_future = None
        self.echo_request_deadline_s = -math.inf
        self.echo_request_next_s = -math.inf
        self.target_echo_interval_confirmed = False
        self.target_echo_interval_result = None
        self.last_sent_target = None
        self.last_sent_s = None
        self.tone_sequence = 0
        self.tone_policy = FollowTonePolicy(float(param("follow_active_tone_repeat_s")),
                                           float(param("landing_active_tone_repeat_s")))
        self.tone_previous_armed = False
        self.tag_tone_suppressed_after_land = False
        self.tone_events = ()
        self.follow_active = self.landing_active = False
        self.tone_event_counts = {event.value: 0 for event in FollowToneEvent}
        self.tone_publisher = self.create_publisher(Mavlink, str(param("tone_mavlink_sink_topic")), 10)
        self.echo_client = self.create_client(CommandLong, str(param("mavros_command_service")))
        self.create_subscription(PositionTarget, str(param("target_echo_topic")),
                                 self._target_echo, qos_profile_sensor_data)

    def _target_echo(self, message):
        self.echo_monitor.receive(self._now_s(),
            (float(message.velocity.x), float(message.velocity.y), float(message.velocity.z)),
            int(message.coordinate_frame), int(message.type_mask))

    def _peripheral_pre_tick(self, now):
        fresh = self._fresh(self.state_received_s, self.state_maximum_age_s, now)
        connected = fresh and self.vehicle_state.connected
        if not connected:
            if self.echo_link_connected:
                self.echo_request_epoch += 1  # Old asynchronous ACKs cannot restore readiness.
                self.echo_future = None
                self.target_echo_interval_confirmed = False
                self.target_echo_interval_result = None
                self.echo_request_next_s = -math.inf
                self.echo_monitor.reset()
                self.last_sent_target = self.last_sent_s = None
        self.echo_link_connected = connected
        if connected:
            self._ensure_target_echo_interval(now)
        due = self.orphan_recovery.update(now, fresh=fresh,
            connected=connected, armed=self.vehicle_state.armed,
            mode=self.vehicle_state.mode.strip().upper(),
            owned=self.lifecycle.active or self.lifecycle.retaining_control)
        enabled = self.output_enabled and bool(self.get_parameter("rollback_orphaned_guided").value)
        self._orphan_rollback_due = due and enabled
        if self._orphan_rollback_due:
            self.pilot_session.invalidate("ORPHANED_GUIDED_ROLLBACK", now)
            self._request_mode(self.lifecycle.fallback_mode, now)
        return enabled and self.orphan_recovery.pending

    def _ensure_target_echo_interval(self, now):
        if not self.allow_target_echo_request or self.target_echo_interval_confirmed:
            return
        if self.echo_future is not None:
            if now < self.echo_request_deadline_s:
                return
            self.echo_request_epoch += 1
            self.echo_future = None
            self.target_echo_interval_result = None
        if now < self.echo_request_next_s or not self.echo_client.service_is_ready():
            return
        retry = float(self.get_parameter("target_echo_request_retry_s").value)
        request = CommandLong.Request()
        request.command = 511  # MAV_CMD_SET_MESSAGE_INTERVAL
        request.param1 = float(self.get_parameter("target_echo_message_id").value)
        request.param2 = float(self.get_parameter("target_echo_interval_us").value)
        epoch = self.echo_request_epoch
        self.echo_request_next_s = self.echo_request_deadline_s = now + retry
        self.echo_future = self.echo_client.call_async(request)
        self.echo_future.add_done_callback(lambda future: self._echo_interval_result(future, epoch))

    def _echo_interval_result(self, future, epoch):
        if epoch != self.echo_request_epoch:
            return
        self.echo_future = None
        try:
            result = future.result()
            self.target_echo_interval_confirmed = bool(result.success)
            self.target_echo_interval_result = int(result.result)
        except Exception:
            self.target_echo_interval_confirmed = False
            self.target_echo_interval_result = None

    def _update_tones(self, snapshot, status, now):
        fresh = self._fresh(self.state_received_s, self.state_maximum_age_s, now)
        connected = fresh and self.vehicle_state.connected
        armed = connected and self.vehicle_state.armed
        mode = self.vehicle_state.mode.strip().upper()
        if armed and not self.tone_previous_armed:
            self.tag_tone_suppressed_after_land = False
        if connected and mode == self.land_mode:
            self.tag_tone_suppressed_after_land = True
        self.tone_previous_armed = armed
        active = armed and snapshot.authorized and self.output_enabled and self.lifecycle.active
        sent = self._fresh(self.last_sent_s, self.echo_monitor.timeout_s, now)
        guided_output = mode == self.guided_mode and sent and self.echo_monitor.fresh(now)
        follow = active and status.action == ActionKind.FOLLOW.value and guided_output
        descending = (status.detail == "GUIDED_TRACK_DESCENT" and guided_output
                      and self.last_sent_target is not None and self.last_sent_target[0][2] < 0)
        landing = (active and status.action in {ActionKind.LAND.value, ActionKind.PRECISION_LAND.value}
                   and (mode == self.land_mode or descending))
        self.follow_active, self.landing_active = bool(follow), bool(landing)
        self.tone_events = self.tone_policy.update(
            observe_ready=bool(armed and snapshot.candidate_fresh
                and mode not in {self.guided_mode,self.land_mode}
                and not self.tag_tone_suppressed_after_land),
            follow_active=bool(follow), landing_active=bool(landing),
            exit_confirmed=bool(connected and (not armed or mode not in {self.guided_mode,self.land_mode})),
            now_s=now)
        self._emit_tones(self.tone_events)

    def _emit_tones(self, events):
        for event in events:
            self.tone_event_counts[event.value] += 1
            if not self.tone_output_enabled:
                continue
            param = lambda name: int(self.get_parameter(name).value)
            frame = encode_legacy_play_tune(TUNES[event], sequence=self.tone_sequence,
                source_system=param("tone_source_system"), source_component=param("tone_source_component"),
                target_system=param("tone_target_system"), target_component=param("tone_target_component"))
            self.tone_sequence = (self.tone_sequence + 1) & 255
            message = Mavlink()
            message.header.stamp = self.get_clock().now().to_msg()
            message.framing_status = Mavlink.FRAMING_OK
            message.magic, message.len = MAVLINK_V2_MAGIC, frame.payload_length
            message.seq, message.sysid, message.compid = frame.sequence, frame.source_system, frame.source_component
            message.msgid, message.checksum = PLAY_TUNE_MSG_ID, frame.checksum
            message.payload64 = list(frame.payload64)
            self.tone_publisher.publish(message)

    def _peripheral_status(self, now):
        status = self.echo_monitor.status(now, self.last_sent_target, self.last_sent_s)
        status.update(
            target_echo_interval_requested=self.target_echo_interval_confirmed,
            target_echo_interval_pending=self.echo_future is not None,
            target_echo_interval_result=self.target_echo_interval_result,
            target_echo_request_enabled=self.allow_target_echo_request,
            target_echo_interval_us=float(self.get_parameter("target_echo_interval_us").value),
            target_echo_message_id=int(self.get_parameter("target_echo_message_id").value),
            orphaned_guided_state=self.orphan_recovery.reason,
            orphaned_guided_rollback_pending=self._orphan_rollback_due,
            tone_output_enabled=self.tone_output_enabled,
            tone_transport="MAVLINK_PLAY_TUNE_LEGACY",
            tone_events=[event.value for event in self.tone_events],
            tone_event_counts=dict(self.tone_event_counts),
            follow_active=self.follow_active,
            landing_active=self.landing_active,
            latest_sent_velocity_mps=(dict(zip(("x","y","z"), self.last_sent_target[0]))
                if self.last_sent_target and self._fresh(self.last_sent_s, self.echo_monitor.timeout_s, now) else None),
        )
        return status
