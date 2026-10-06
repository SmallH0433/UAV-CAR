"""Vertical observations and final-writer inhibition; never a second controller."""
import json
import math

from air_ground_landing.action_execution import ActionCommand
from air_ground_landing.vertical_latch import VerticalFaultLatch
from air_ground_landing.vertical_safety import (
    VerticalSafetyMonitor, VerticalSafetyConfig, VerticalSample,
    HeightReferenceController, HeightReferenceInput,
)


VERTICAL_DEFAULTS = {
    "vertical_guard_mode": "shadow",
    "vertical_guard_state_path": "/state/vertical_guard.json",
    "vertical_guard_transition_s": 0.3,
    "vertical_guard_reset_dwell_s": 3.0,
    "vertical_guard_reset_topic": "/landing/vertical_guard/reset",
    "range_status_topic": "/landing/sensor_range/status",
}


def finite(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


class VerticalGuard:
    def _init_vertical_guard(self):
        self.vertical_guard_mode = str(self.get_parameter("vertical_guard_mode").value).lower()
        if self.vertical_guard_mode not in {"shadow", "enforce"}:
            raise ValueError("vertical_guard_mode must be shadow or enforce")
        self.vertical_transition_s = float(self.get_parameter("vertical_guard_transition_s").value)
        self.vertical_reset_dwell_s = float(self.get_parameter("vertical_guard_reset_dwell_s").value)
        if not finite(self.vertical_transition_s) or not 0 <= self.vertical_transition_s <= .5:
            raise ValueError("vertical transition must be in [0, 0.5] seconds")
        if not finite(self.vertical_reset_dwell_s) or self.vertical_reset_dwell_s < 1:
            raise ValueError("vertical reset dwell must be at least 1 second")
        self.vertical_latch = VerticalFaultLatch(self.get_parameter("vertical_guard_state_path").value)
        self.vertical_monitor = VerticalSafetyMonitor(VerticalSafetyConfig(
            vertical_status_max_age_s=self.ekf_maximum_age_s))
        self.height_reference = HeightReferenceController()
        self.vertical_health = None
        self.vertical_height_shadow = None
        self.vertical_inhibited = self.vertical_latch.latched
        self.vertical_inhibit_reason = self.vertical_latch.reason
        self.vertical_transition_started_s = None  # Restarts never replay the transition.
        self.vertical_reset_ready_since_s = None
        self.vertical_reset_result = "NOT_REQUESTED"
        self.vertical_reset_gate_reason = "NOT_EVALUATED"
        self.vertical_native_land_confirmed = False
        self.vertical_pilot_mode_observed = False
        self.vertical_source_stamps = {}
        self.vertical_source_times = {}
        self.vertical_source_errors = {}
        self.vertical_last_valid_times = {}
        self.vertical_pose = None
        self.vertical_pose_history = []
        self.vertical_velocity = None
        self.vertical_vehicle_state = None
        self.vertical_extended = None
        self.vertical_rc_channels = None
        self.vertical_status = {}
        self.vertical_status_time_s = None
        self.vertical_ekf_seq = None
        self.vertical_range_status = {}
        self.vertical_range_sensor_time = None
        self.vertical_range_rebase_count = None
        self.vertical_range_measurement_s = None
        self.vertical_range_min_m = None
        self.vertical_range_max_m = None
        self.vertical_pnp = {}
        self.vertical_pnp_last_frame = None
        self.vertical_pnp_reason = "NO_CAPTURE_EVIDENCE"
        self.vertical_previous_session = None
        self.vertical_ground_samples = []
        self.vertical_ground_health_s = None
        self.vertical_ground_health_reason = "NOT_OBSERVED"

    def _vertical_source_stamp(self, source, message, now):
        """Convert a ROS source stamp to local monotonic time; no receipt fallback.

        A re-published identical stamp cannot refresh evidence. Zero, future,
        and backward stamps invalidate the current source instead of becoming green.
        """
        try:
            stamp = message.header.stamp
            raw = int(stamp.sec) + int(stamp.nanosec) * 1e-9
            ros_now = self.get_clock().now().nanoseconds * 1e-9
        except (AttributeError, TypeError, ValueError):
            raw, ros_now = math.nan, math.nan
        previous = self.vertical_source_stamps.get(source)
        if not finite(raw) or raw <= 0 or not finite(ros_now) or raw > ros_now + .02:
            self.vertical_source_errors[source] = "INVALID_SOURCE_STAMP"
            self.vertical_source_times[source] = None
            if source == "pose":
                self.vertical_pose_history.clear()
            return None
        if previous is not None and raw <= previous:
            self.vertical_source_errors[source] = "DUPLICATE_SOURCE" if raw == previous else "SOURCE_CLOCK_BACKWARD"
            if raw < previous:
                self.vertical_source_times[source] = None
                if source == "pose":
                    self.vertical_pose_history.clear()
            return None
        self.vertical_source_stamps[source] = raw
        source_time = now - max(0.0, ros_now - raw)
        self.vertical_source_times[source] = source_time
        self.vertical_source_errors.pop(source, None)
        return source_time

    def _vertical_record_pose(self, message, source_time):
        """Keep validated FLU→ENU attitude with its original measurement time."""
        q = message.pose.orientation
        values = (q.x, q.y, q.z, q.w)
        if not all(finite(value) for value in values):
            return
        norm = math.sqrt(sum(value*value for value in values))
        if not .9 <= norm <= 1.1:
            return
        self.vertical_pose_history.append((source_time, tuple(value/norm for value in values)))
        self.vertical_pose_history[:] = [row for row in self.vertical_pose_history
                                       if 0 <= source_time-row[0] <= 1.0][-64:]

    def _vertical_estimator(self, message, source_time):
        self.vertical_status = {
            "source": "ESTIMATOR_STATUS",
            "vertical_position_valid": bool(message.pos_vert_abs_status_flag or message.pos_vert_agl_status_flag),
            "vertical_velocity_valid": bool(message.velocity_vert_status_flag),
            # MAVROS exposes the standard innovation test ratios here.
            "velocity_ratio": self._vertical_metric(getattr(message, "vel_ratio", None)),
            "vertical_position_ratio": self._vertical_metric(getattr(message, "pos_vert_ratio", None)),
            "height_above_ground_ratio": self._vertical_metric(getattr(message, "hagl_ratio", None)),
        }
        self.vertical_status_time_s = source_time

    @staticmethod
    def _vertical_metric(value):
        return float(value) if finite(value) and value >= 0 else None

    def _vertical_range_status(self, message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError("range status must be an object")
        except (TypeError, ValueError):
            self.vertical_range_status = {}
            return
        now = self._now_s()
        # Existing range bridge has a sensor boot clock, age and reset count.
        # Only a new original sensor tick anchors measurement time. Repeated
        # status publications cannot make an old value fresh.
        sensor_time = data.get("sensor_time_ms")
        rebase = data.get("rebase_count")
        age = data.get("age_s")
        identity_ok = data.get("source") == "200/88" and data.get("id") == 0 and data.get("orientation") == 25
        if (not identity_ok or not finite(sensor_time) or not finite(rebase)
                or not finite(age) or age < 0):
            self.vertical_range_status = {}
            return
        if self.vertical_range_rebase_count is not None and rebase != self.vertical_range_rebase_count:
            self.vertical_range_status = {}
            self.vertical_range_sensor_time = sensor_time
            self.vertical_range_rebase_count = rebase
            self.vertical_range_measurement_s = None
            self.vertical_source_times["range"] = None
            self.vertical_source_errors["range"] = "RANGE_CLOCK_REBASE"
            return
        self.vertical_range_rebase_count = rebase
        if self.vertical_range_sensor_time is not None and sensor_time < self.vertical_range_sensor_time:
            self.vertical_range_status = {}
            self.vertical_range_measurement_s = None
            self.vertical_source_times["range"] = None
            self.vertical_source_errors["range"] = "RANGE_SENSOR_CLOCK_BACKWARD"
            return
        if sensor_time != self.vertical_range_sensor_time:
            stamp = data.get("stamp_monotonic_s", now)
            measured = data.get("measurement_monotonic_s", now-age)
            if not finite(stamp) or not finite(measured) or not measured <= stamp <= now+.02:
                self.vertical_range_status = {}
                return
            self.vertical_range_measurement_s = measured
            self.vertical_range_sensor_time = sensor_time
            self.vertical_source_errors.pop("range", None)
        elif self.vertical_range_status:
            # Preserve the whole accepted sample. A repeated sensor id with a
            # changed value must not smuggle a new observation into the window.
            measured_value = self.vertical_range_status.get("range_m")
            if data.get("range_m") != measured_value:
                self.vertical_range_status = {}
                self.vertical_source_errors["range"] = "RANGE_DUPLICATE_VALUE_CHANGED"
                return
        normalized = dict(data, stamp_monotonic_s=now,
                          measurement_monotonic_s=self.vertical_range_measurement_s)
        if "quality" not in normalized and data.get("signal_quality") is not None:
            signal = data["signal_quality"]
            normalized["quality"] = (None if signal == 0 else 0.0 if signal == 1
                                      else signal / 100.0 if finite(signal) else math.nan)
        self.vertical_range_status = normalized

    def _vertical_accept_pnp(self, payload, now):
        # Accepted frame identity/time stay immutable across polling gaps. A
        # cached frame can remain evidence until its ORIGINAL age expires, but
        # never adds independent samples or persistence by being polled again.
        if payload.get("accepted_this_poll") is not True:
            return
        stamp = payload.get("accepted_capture_time_s")
        frame = payload.get("accepted_frame_id")
        target = payload.get("accepted_target_num")
        if (payload.get("accepted_time_basis") != "libcamera_sensor_timestamp"
                or payload.get("accepted_capture_timing_valid") is not True):
            self.vertical_pnp = {}
            self.vertical_pnp_reason = "CAPTURE_TIMESTAMP_UNVERIFIED"
            return
        position = payload.get("accepted_body_frd_m")
        if (not isinstance(frame, int) or isinstance(frame, bool) or frame < 0
                or not isinstance(target, int) or isinstance(target, bool) or target < 0
                or not finite(stamp) or not 0 <= now - stamp <= .35
                or payload.get("accepted_body_frame") != "BODY_FRD"
                or not isinstance(position, (list, tuple)) or len(position) != 3
                or not all(finite(value) for value in position)):
            self.vertical_pnp = {}
            self.vertical_pnp_reason = "CAPTURE_POSITION_OR_TIME_INVALID"
            return
        if frame == self.vertical_pnp_last_frame:
            return
        previous = self.vertical_source_times.get("pnp")
        if previous is not None and stamp <= previous:
            self.vertical_pnp = {}
            self.vertical_pnp_reason = "CAPTURE_TIME_NOT_INCREASING"
            return
        if not self.vertical_pose_history:
            self.vertical_pnp = {}
            self.vertical_pnp_reason = "CAPTURE_ATTITUDE_UNAVAILABLE"
            return
        attitude_time, quaternion = min(self.vertical_pose_history, key=lambda row: abs(row[0]-stamp))
        if abs(attitude_time-stamp) > .1+1e-9:
            self.vertical_pnp = {}
            self.vertical_pnp_reason = "CAPTURE_ATTITUDE_NOT_ALIGNED"
            return
        # MAVROS local pose rotates BODY_FLU into ENU. Convert the complete
        # FRD vector first; its down component alone is not ground height.
        vx, vy, vz = position[0], -position[1], -position[2]
        x, y, z, w = quaternion
        enu_z = 2*(x*z-w*y)*vx + 2*(y*z+w*x)*vy + (1-2*(x*x+y*y))*vz
        height = -enu_z
        if not finite(height) or height <= 0:
            self.vertical_pnp = {}
            self.vertical_pnp_reason = "CAPTURE_NOT_BELOW_VEHICLE"
            return
        self.vertical_pnp_last_frame = frame
        self.vertical_source_times["pnp"] = stamp
        self.vertical_pnp = dict(height=height, time=stamp,
                                 source=str(target), attitude_time=attitude_time,
                                 time_basis="libcamera_sensor_timestamp")
        self.vertical_pnp_reason = "CAPTURE_TIMESTAMP_AND_ATTITUDE_VALID"

    def _vertical_observe_state(self, message):
        mode = message.mode.strip().upper()
        if message.connected and mode == self.land_mode:
            self.vertical_native_land_confirmed = True
        elif message.connected and mode != self.land_mode:
            self.vertical_native_land_confirmed = False
        if self.vertical_inhibited and mode != self.guided_mode:
            self.vertical_pilot_mode_observed = True
            self.vertical_transition_started_s = None

    def _vertical_sample(self, now):
        p = self.vertical_pose.pose if self.vertical_pose is not None else None
        v = self.vertical_velocity.twist if self.vertical_velocity is not None else None
        roll = pitch = None
        if p is not None:
            q = p.orientation
            norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            if finite(norm) and .9 <= norm <= 1.1:
                x,y,z,w = q.x/norm,q.y/norm,q.z/norm,q.w/norm
                roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
                pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
        status = self.vertical_range_status
        measured = status.get("measurement_monotonic_s")
        return VerticalSample(
            now_s=now, z_m=p.position.z if p else None, vz_mps=v.linear.z if v else None,
            pose_time_s=self.vertical_source_times.get("pose"),
            velocity_time_s=self.vertical_source_times.get("velocity"),
            range_m=status.get("range_m"), range_time_s=measured,
            range_min_m=status.get("min_range_m", self.vertical_range_min_m),
            range_max_m=status.get("max_range_m", self.vertical_range_max_m),
            range_healthy=status.get("healthy") is True,
            range_status_time_s=status.get("stamp_monotonic_s"),
            range_quality=(status.get("quality") if finite(status.get("quality")) else math.nan)
                if "quality" in status and status["quality"] is not None else None,
            roll_rad=roll, pitch_rad=pitch,
            attitude_time_s=self.vertical_source_times.get("pose"),
            pnp_height_m=self.vertical_pnp.get("height"),
            pnp_time_s=self.vertical_pnp.get("time"),
            pnp_source_id=self.vertical_pnp.get("source"), pnp_accepted=bool(self.vertical_pnp),
            vertical_position_valid=self.vertical_status.get("vertical_position_valid"),
            vertical_velocity_valid=self.vertical_status.get("vertical_velocity_valid"),
            vertical_status_time_s=self.vertical_status_time_s,
            mode=self.vehicle_state.mode,
            commanded_vz_mps=self._vertical_sent_vz(now),
            active=True, context="vertical_observations",
        )

    def _vertical_update(self, now):
        self.vertical_health = self.vertical_monitor.update(self._vertical_sample(now))
        health = self.vertical_health
        for source, stamp in health.source_times.items():
            if finite(stamp):
                self.vertical_last_valid_times[source] = stamp
        state = health.state.value if hasattr(health.state, "value") else health.state
        native_land = self.vertical_native_land_confirmed
        if self.vertical_guard_mode == "enforce" and state == "FAULT":
            self.vertical_latch.trip(health.reason)
        owns = self.lifecycle.active or self.lifecycle.retaining_control
        if self.vertical_latch.latched or (
                self.vertical_guard_mode == "enforce" and state != "HEALTHY" and owns and not native_land):
            self._vertical_inhibit(now, self.vertical_latch.reason or health.reason)
        session = self.pilot_session.session_sequence
        if self.vertical_previous_session != session:
            self.height_reference.reset()
            self.vertical_previous_session = session
        request = self.lifecycle.request
        self.vertical_height_shadow = self.height_reference.update(HeightReferenceInput(
            now_s=now, health=health.state, height_m=health.trusted_height_m,
            height_time_s=health.source_times.get("range"),
            action=request.kind.value if request and self.lifecycle.active else "IDLE",
            phase=self.lifecycle.land_phase,
            tag_fresh=self.target_healthy and self._fresh(self.target_status_received_s, self.candidate_maximum_age_s, now),
            nominal_descent_mps=self.lifecycle.land_guided_descent_mps,
            session_id=str(session),
        ))
        # The health dwell must cover every ground/reset gate continuously.
        if self._vertical_reset_conditions(now) and self._vertical_ground_healthy(now):
            if self.vertical_reset_ready_since_s is None:
                self.vertical_reset_ready_since_s = now
            if (not self.vertical_inhibited and not self.vertical_latch.latched
                    and now-self.vertical_reset_ready_since_s >= self.vertical_reset_dwell_s):
                if not self.vertical_latch.end_ground_session():
                    self._vertical_inhibit(now, "VERTICAL_GROUND_SESSION_PERSISTENCE_FAILED")
        else:
            self.vertical_reset_ready_since_s = None
            self.vertical_ground_samples.clear()

    def _vertical_begin_session(self, now):
        # Shadow observations do not create an automatic-session marker. A
        # marker/fault from an earlier enforce session still blocks shadow.
        if self.vertical_guard_mode == "shadow":
            return not self.vertical_inhibited and not self.vertical_latch.latched
        if self.vertical_latch.begin_session():
            return True
        self._vertical_inhibit(now, self.vertical_latch.reason)
        # The first actuator request has not been sent. Do not send transition
        # setpoints after a failed preflight durability check either.
        self.vertical_transition_started_s = None
        return False

    def _vertical_inhibit(self, now, reason):
        if not self.vertical_inhibited:
            owns = self.lifecycle.active or self.lifecycle.retaining_control
            self.vertical_transition_started_s = now if owns and self.vehicle_state.mode.strip().upper() == self.guided_mode else None
        self.vertical_inhibited = True
        self.vertical_inhibit_reason = reason
        self.pilot_session.invalidate("VERTICAL_GUARD_INHIBITED", now)
        self.owned_mode_deadlines.clear()
        # A MAVLink request already handed to FC cannot be recalled; discard
        # its completion and never retry or treat a late heartbeat as rearm.
        if self.mode_future is not None and not self.mode_future.done():
            self.mode_future.cancel()
        self._orphan_rollback_due = False
        # A heartbeat-confirmed native LAND keeps FC ownership. Never re-enter a
        # historical GUIDED/fallback state because a visual/vertical source vanished.
        if not self.vertical_native_land_confirmed:
            if self.lifecycle.active or self.lifecycle.retaining_control:
                self.lifecycle.cancel(None, now, "VERTICAL_GUARD_INHIBITED")

    def _vertical_gate_command(self, command, snapshot, now):
        if (self.vertical_guard_mode == "enforce" and not self._vertical_ready_for_new_action(now)
                and not self.vertical_native_land_confirmed):
            self._vertical_inhibit(now, self.vertical_inhibit_reason or "VERTICAL_HEALTH_NOT_READY")
        if not self.vertical_inhibited:
            # Even an old lifecycle command cannot undo confirmed native LAND.
            if (self.vertical_native_land_confirmed and command.desired_mode
                    and command.desired_mode != self.land_mode):
                return None
            return command
        if (self.vertical_transition_started_s is not None
                and (self.vertical_latch.storage_error is None or self.vertical_latch.session_active)
                and not self.vertical_native_land_confirmed and not self.vertical_pilot_mode_observed
                and 0 <= now-self.vertical_transition_started_s < self.vertical_transition_s
                and snapshot.mode.strip().upper() == self.guided_mode
                and snapshot.connected and snapshot.armed and snapshot.telemetry_fresh
                and self._vertical_rc_high(now)):
            # Limited horizontal braking request; zero Vz is NOT a height guarantee.
            return ActionCommand(velocity_enu=(0., 0., 0.), yaw_rate_rad_s=0.)
        return None

    def _vertical_ready_for_new_action(self, now):
        return (not self.vertical_inhibited and (self.vertical_guard_mode == "shadow"
            or (self.vertical_health is not None and self.vertical_health.state == "HEALTHY"
                and self._fresh(self.vertical_health.time_s, .2, now))))

    def _vertical_rc_high(self, now):
        index = int(self.get_parameter("rc_channel").value)-1
        channels = self.vertical_rc_channels
        return bool(channels is not None and 0 <= index < len(channels)
            and self._fresh(self.vertical_source_times.get("rc"), float(self.get_parameter("rc_maximum_age_s").value), now)
            and channels[index] >= int(self.get_parameter("rc_authorize_above_pwm").value))

    def _vertical_reset_conditions(self, now):
        index = int(self.get_parameter("rc_channel").value)-1
        state, extended, channels = self.vertical_vehicle_state, self.vertical_extended, self.vertical_rc_channels
        gates = (
            (state is not None and self._fresh(self.vertical_source_times.get("state"), self.state_maximum_age_s, now),
             "FC_STATE_MISSING_OR_STALE"),
            (state is not None and state.connected, "FC_NOT_CONNECTED"),
            (state is not None and not state.armed, "FC_DISARMED_REQUIRED"),
            (extended is not None and self._fresh(self.vertical_source_times.get("extended"), self.state_maximum_age_s, now)
                and extended.landed_state == 1, "ON_GROUND_NOT_CONFIRMED"),
            (channels is not None and 0 <= index < len(channels)
                and self._fresh(self.vertical_source_times.get("rc"), float(self.get_parameter("rc_maximum_age_s").value), now),
             "RC_SIGNAL_MISSING_OR_STALE"),
            (channels is not None and 0 <= index < len(channels)
                and 800 <= channels[index] <= int(self.get_parameter("rc_abort_below_pwm").value), "RC6_MUST_BE_LOW"),
        )
        for valid, reason in gates:
            if not valid:
                self.vertical_reset_gate_reason = reason
                return False
        self.vertical_reset_gate_reason = "GROUND_INTERLOCKS_VALID_WAIT_STABLE_SOURCES"
        return True

    def _vertical_ground_healthy(self, now):
        """Independent reset-only ground check, including ranges below flight envelope.

        Does not turn airborne UNKNOWN into HEALTHY or authorize a flight.
        """
        sample = self._vertical_sample(now)
        values = (sample.range_m, sample.z_m, sample.vz_mps, sample.range_min_m, sample.range_max_m)
        valid = (all(finite(x) for x in values)
            and sample.range_healthy is True and 0 < sample.range_min_m <= sample.range_m <= sample.range_max_m
            and self._fresh(sample.range_time_s, .2, now)
            and self._fresh(sample.pose_time_s, .2, now) and self._fresh(sample.velocity_time_s, .2, now)
            and finite(sample.roll_rad) and finite(sample.pitch_rad)
            and abs(sample.roll_rad) <= math.radians(10) and abs(sample.pitch_rad) <= math.radians(10)
            and self._fresh(sample.range_status_time_s, .5, now)
            and self._fresh(sample.vertical_status_time_s, self.ekf_maximum_age_s, now)
            and sample.vertical_position_valid is True and sample.vertical_velocity_valid is True
            and abs(sample.vz_mps) <= .05
            and (sample.range_quality is None or finite(sample.range_quality) and 0 < sample.range_quality <= 1))
        if not valid:
            self.vertical_ground_health_reason = "GROUND_SOURCES_OR_FLAGS_INVALID"
            self.vertical_ground_health_s = None
            return False
        rows = self.vertical_ground_samples
        if rows and (now < rows[-1][0] or now-rows[-1][0] > .2):
            rows.clear()
            self.vertical_reset_ready_since_s = None
        rows.append((now, sample.range_m, sample.z_m))
        rows[:] = [r for r in rows if now-r[0] <= self.vertical_reset_dwell_s+.2]
        stable = max(r[1] for r in rows)-min(r[1] for r in rows) <= .02 and max(r[2] for r in rows)-min(r[2] for r in rows) <= .03
        self.vertical_ground_health_s = now if stable else None
        self.vertical_ground_health_reason = "GROUND_SOURCES_STABLE" if stable else "GROUND_MEASUREMENTS_MOVING"
        return stable

    def _vertical_reset(self, message):
        now = self._now_s()
        try:
            request = json.loads(message.data)
        except (TypeError, ValueError):
            request = None
        if request != {"reset": True}:
            self.vertical_reset_result = "REJECTED_EXPLICIT_RESET_REQUIRED"
        elif not (self._vertical_reset_conditions(now)
                  and self.vertical_reset_ready_since_s is not None
                  and 0 <= now-self.vertical_reset_ready_since_s
                  and now-self.vertical_reset_ready_since_s >= self.vertical_reset_dwell_s
                  and self._vertical_ground_healthy(now)
                  and self.vertical_ground_samples[-1][0]-self.vertical_ground_samples[0][0] >= self.vertical_reset_dwell_s
                  and self._fresh(self.vertical_ground_health_s, .2, now)):
            self.vertical_reset_result = "REJECTED_REQUIRES_FRESH_DISARMED_ON_GROUND_RC6_LOW_STABLE_HEALTH"
        elif not self.vertical_latch.clear():
            self.vertical_reset_result = "REJECTED_PERSISTENCE_FAILED"
        else:
            self.vertical_inhibited = False
            self.vertical_inhibit_reason = ""
            self.vertical_transition_started_s = None
            self.vertical_pilot_mode_observed = False
            self.height_reference.reset()
            self.pilot_session.invalidate("VERTICAL_GROUND_RESET_NEW_RC_AUTH_REQUIRED", now)
            self.vertical_reset_result = "RESET_NEW_RC_AUTH_REQUIRED"
        self._publish_event(dict(event="VERTICAL_RESET", **self._vertical_status_payload(now)))

    def _vertical_sent_vz(self, now):
        if (self.last_sent_target and self._fresh(self.last_sent_s, self.echo_monitor.timeout_s, now)
                and not self.last_sent_target[2] & 32):
            value = self.last_sent_target[0][2]
            return value if finite(value) else None
        return None

    def _vertical_status_payload(self, now):
        health = self.vertical_health.to_dict() if self.vertical_health else {}
        shadow = self.vertical_height_shadow.to_dict() if self.vertical_height_shadow else {}
        stage = "monitoring"
        if self.vertical_inhibited:
            stage = "native_land_fcu_owned" if self.vertical_native_land_confirmed else "await_pilot"
            if (self.vertical_transition_started_s is not None
                    and not self.vertical_native_land_confirmed
                    and 0 <= now-self.vertical_transition_started_s < self.vertical_transition_s):
                stage = "limited_zero_velocity_transition"
        return dict(vertical_guard_mode=self.vertical_guard_mode,
            vertical_state=health.get("state", "UNKNOWN"),
            vertical_fault_latched=self.vertical_latch.latched,
            vertical_fault_reason=self.vertical_latch.reason or None,
            vertical_guard_inhibited=self.vertical_inhibited,
            vertical_guard_reason=self.vertical_inhibit_reason or health.get("reason"),
            vertical_guard_action=stage, vertical_safe_hover_guaranteed=False,
            vertical_health=health, vertical_last_valid_source_times=dict(self.vertical_last_valid_times),
            vertical_source_errors=dict(self.vertical_source_errors),
            vertical_estimator=dict(self.vertical_status),
            vertical_pnp_time_basis=self.vertical_pnp.get("time_basis"),
            vertical_pnp_capture_time_s=self.vertical_pnp.get("time"),
            vertical_pnp_attitude_time_s=self.vertical_pnp.get("attitude_time"),
            vertical_pnp_height_m=self.vertical_pnp.get("height")
                if self._fresh(self.vertical_pnp.get("time"), .35, now) else None,
            vertical_pnp_evidence_reason=("CAPTURE_EXPIRED" if self.vertical_pnp
                and not self._fresh(self.vertical_pnp.get("time"), .35, now) else self.vertical_pnp_reason),
            vertical_height_reference_shadow=shadow,
            vertical_latch_storage_error=self.vertical_latch.storage_error,
            vertical_session_durable=self.vertical_latch.session_active,
            vertical_ground_health_reason=self.vertical_ground_health_reason,
            vertical_mode_request_may_be_in_flight=self.mode_request_action_id is not None
                and self.vertical_inhibited,
            vertical_reset_result=self.vertical_reset_result,
            vertical_reset_gate_reason=self.vertical_reset_gate_reason,
            vertical_reset_ready_since_s=self.vertical_reset_ready_since_s,
            commanded_vertical_speed_mps=self._vertical_sent_vz(now))
