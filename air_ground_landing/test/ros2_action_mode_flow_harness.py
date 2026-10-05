#!/usr/bin/env python3
"""Exercise the RC6/RC8 landing and tag-loss flow against fake MAVROS."""

import json
import sys
import time
import os

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import (
    EstimatorStatus,
    ExtendedState,
    HomePosition,
    PositionTarget,
    RCIn,
    State,
    Mavlink,
)
from mavros_msgs.srv import SetMode, CommandLong
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import String


class ModeFlowHarness(Node):
    def __init__(self) -> None:
        super().__init__("action_mode_flow_harness")
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.mode = os.environ.get("HARNESS_START_MODE", "LOITER")
        self.pending_mode = None
        self.pending_mode_deadline_s = 0.0
        self.ch6_pwm = 1000
        self.ch8_pwm = 1000
        self.tag_visible = True
        self.tag_heading_error_rad = -.4
        self.tag_center_error_px = 70.0
        self.range_m = 0.8
        self.mode_requests = []
        self.statuses = []
        self.interval_requests = []
        self.tones = []
        self.last_setpoint = None
        self.state_pub = self.create_publisher(State, "/mavros/state", latched)
        self.rc_pub = self.create_publisher(RCIn, "/mavros/rc/in", 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, "/mavros/local_position/pose", 10
        )
        self.velocity_pub = self.create_publisher(
            TwistStamped, "/mavros/local_position/velocity_local", 10
        )
        self.extended_pub = self.create_publisher(
            ExtendedState, "/mavros/extended_state", 10
        )
        self.estimator_pub = self.create_publisher(
            EstimatorStatus, "/mavros/estimator_status", 10
        )
        self.home_pub = self.create_publisher(
            HomePosition, "/mavros/home_position/home", latched
        )
        self.candidate_pub = self.create_publisher(
            PositionTarget, "/landing/ibvs/candidate", 10
        )
        self.ibvs_status_pub = self.create_publisher(
            String, "/landing/ibvs/status", 10
        )
        self.target_status_pub = self.create_publisher(
            String, "/landing/landing_target/status", 10
        )
        self.range_pub = self.create_publisher(
            Range, "/mavros/distance_sensor/rangefinder_pub", 10
        )
        self.create_subscription(
            String, "/landing/action/status", self._status, 10
        )
        self.create_service(SetMode, "/mavros/set_mode", self._set_mode)
        self.create_service(CommandLong, "/mavros/cmd/command", self._command_long)
        self.echo_pub = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/target_local", 10)
        self.create_subscription(PositionTarget, "/mavros/setpoint_raw/local", self._setpoint, 10)
        self.create_subscription(Mavlink, "/uas1/mavlink_sink", self.tones.append, 10)
        self.create_timer(0.05, self._telemetry)

    def _setpoint(self, message):
        self.last_setpoint = message

    def _command_long(self, request, response):
        if request.command == 511:
            self.interval_requests.append((request.param1, request.param2))
            response.success = request.param1 == 85 and request.param2 == 200000
            response.result = 0 if response.success else 2
        else:
            response.success = False
            response.result = 2
        return response

    def _set_mode(self, request, response):
        self.mode_requests.append(request.custom_mode)
        # SetMode acknowledges transport before the mode heartbeat changes,
        # matching the asynchronous behavior of a real FCU.
        self.pending_mode = request.custom_mode
        self.pending_mode_deadline_s = time.monotonic() + 0.25
        response.mode_sent = True
        return response

    def _status(self, message: String) -> None:
        self.statuses.append(json.loads(message.data))

    def _telemetry(self) -> None:
        if (
            self.pending_mode is not None
            and time.monotonic() >= self.pending_mode_deadline_s
        ):
            self.mode = self.pending_mode
            self.pending_mode = None
        state = State()
        state.connected = True
        state.armed = True
        state.mode = self.mode
        self.state_pub.publish(state)
        if self.mode == "GUIDED" and self.last_setpoint is not None:
            self.echo_pub.publish(self.last_setpoint)

        rc = RCIn()
        rc.channels = [1500] * 8
        rc.channels[5] = self.ch6_pwm
        rc.channels[7] = self.ch8_pwm
        self.rc_pub.publish(rc)

        pose = PoseStamped()
        pose.pose.position.z = 1.0
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        self.velocity_pub.publish(TwistStamped())

        estimator = EstimatorStatus()
        estimator.attitude_status_flag = True
        estimator.velocity_horiz_status_flag = True
        estimator.pos_horiz_rel_status_flag = True
        self.estimator_pub.publish(estimator)
        self.home_pub.publish(HomePosition())

        extended = ExtendedState()
        extended.landed_state = ExtendedState.LANDED_STATE_IN_AIR
        self.extended_pub.publish(extended)

        if self.tag_visible:
            self.candidate_pub.publish(PositionTarget())
        ibvs = String()
        ibvs.data = json.dumps({
            "healthy": self.tag_visible,
            "aligned": self.tag_visible,
            "landing_alignment": {
                "frame": "BODY_FLU", "velocity_flu": [.08, -.04],
                "heading_error_rad": self.tag_heading_error_rad,
                "center_error_px": self.tag_center_error_px,
                "source_age_s": 0.0,
            } if self.tag_visible else None,
        })
        self.ibvs_status_pub.publish(ibvs)

        target = String()
        target.data = json.dumps({
            "stream_healthy": self.tag_visible,
            "output_enabled": True,
        })
        self.target_status_pub.publish(target)

        distance = Range()
        distance.min_range = 0.02
        distance.max_range = 8.0
        distance.range = self.range_m
        self.range_pub.publish(distance)


def wait_for(node, predicate, timeout_s: float, label: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    latest = node.statuses[-1] if node.statuses else None
    raise TimeoutError(f"{label} was not reached; latest={latest}")


def status_seen(node, *, action=None, state=None, detail=None, reason=None):
    return any(
        (action is None or item.get("action") == action)
        and (state is None or item.get("state") == state)
        and (detail is None or item.get("detail") == detail)
        and (reason is None or item.get("reason") == reason)
        for item in node.statuses
    )


def spin_for(node, duration_s: float) -> None:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def main() -> int:
    rclpy.init()
    node = ModeFlowHarness()
    try:
        # On the Pi, executor startup/discovery can outlast a fixed sleep.
        # Begin the required RC-low dwell only after its status is observable.
        wait_for(node, lambda: bool(node.statuses), 20.0, "executor discovery")
        if os.environ.get("HARNESS_START_MODE") == "GUIDED":
            wait_for(node, lambda: node.mode == "LOITER", 8.0, "startup orphan GUIDED rollback")
            if node.last_setpoint is not None:
                raise AssertionError("startup recovery emitted a motion setpoint")
            print("ROS_ORPHANED_GUIDED_RECOVERY_OK")
            return 0
        spin_for(node, 1.0)
        node.ch6_pwm = 2000
        wait_for(
            node,
            lambda: node.mode == "GUIDED"
            and any(item.get("follow_active") for item in node.statuses),
            5.0,
            "CH6 GUIDED follow",
        )
        first_follow_id = next(
            item["action_id"]
            for item in reversed(node.statuses)
            if item.get("action") == "FOLLOW"
            and item.get("state") == "RUNNING"
        )

        node.ch8_pwm = 2000
        wait_for(node, lambda: status_seen(node, action="LAND", detail="GUIDED_ALIGN"),
                 5.0, "CH8 alignment before descent")
        spin_for(node, .6)
        if node.last_setpoint is None or node.last_setpoint.velocity.z != 0:
            raise AssertionError("descent emitted before centering and heading alignment")
        if node.last_setpoint.yaw_rate >= 0:
            raise AssertionError("alignment did not turn toward the tag heading")
        node.tag_heading_error_rad = 0.0
        node.tag_center_error_px = 10.0
        wait_for(node, lambda: status_seen(node, action="LAND", detail="GUIDED_TRACK_DESCENT"),
                 5.0, "CH8 GUIDED descent")
        if "LAND" in node.mode_requests:
            raise AssertionError("native LAND requested above handoff height")
        wait_for(node, lambda: node.last_setpoint is not None
                 and node.last_setpoint.velocity.z < 0
                 and node.last_setpoint.velocity.x > 0
                 and node.last_setpoint.yaw_rate == 0
                 and not (node.last_setpoint.type_mask & PositionTarget.IGNORE_YAW_RATE),
                 3.0, "descent after centering and heading alignment")
        node.statuses.clear()
        node.tag_heading_error_rad = -.4
        wait_for(node, lambda: status_seen(node, detail="GUIDED_ALIGN")
                 and node.last_setpoint.velocity.z == 0,
                 3.0, "misalignment stops descent")
        node.tag_heading_error_rad = 0.0
        node.statuses.clear()
        wait_for(node, lambda: status_seen(node, detail="GUIDED_TRACK_DESCENT"),
                 3.0, "realignment resumes descent")
        node.statuses.clear()
        node.tag_visible = False
        wait_for(node, lambda: status_seen(node, detail="TAG_LOST_GUIDED_HOLD"),
                 5.0, "tag loss hold")
        node.tag_visible = True
        wait_for(node, lambda: status_seen(node, detail="GUIDED_TRACK_DESCENT"),
                 5.0, "reacquire GUIDED descent")
        node.ch8_pwm = 1000
        wait_for(node, lambda: status_seen(node, reason="LAND_EXIT_CH8_RELEASED"),
                 5.0, "CH8 exit")
        wait_for(node, lambda: any(x.get("action") == "FOLLOW" and
                 x.get("state") == "RUNNING" and x.get("action_id") != first_follow_id
                 for x in node.statuses), 5.0, "FOLLOW resume")
        node.statuses.clear()
        node.ch8_pwm = 2000
        wait_for(node, lambda: status_seen(node, detail="GUIDED_TRACK_DESCENT"),
                 5.0, "second descent")
        node.tag_visible = False
        wait_for(node, lambda: node.mode == "LOITER" and status_seen(
                 node, state="DONE", reason="LAND_EXIT_TAG_REACQUIRE_TIMEOUT"),
                 8.0, "recovery timeout")
        node.tag_visible = True
        node.ch8_pwm = 1000
        spin_for(node, .5)
        node.statuses.clear()
        wait_for(node, lambda: node.mode == "GUIDED" and status_seen(
                 node, action="FOLLOW", state="RUNNING"), 5.0, "follow after timeout")
        node.ch8_pwm = 2000
        wait_for(node, lambda: status_seen(node, detail="GUIDED_TRACK_DESCENT"),
                 5.0, "third descent")
        node.range_m = .10
        wait_for(node, lambda: node.mode == "LAND" and status_seen(
                 node, detail="LAND_COMMITTED"), 5.0, "0.10m native LAND handoff")
        if node.mode_requests.count("LAND") != 1:
            raise AssertionError(f"unexpected native LAND requests: {node.mode_requests}")
        if (85.0, 200000.0) not in node.interval_requests:
            raise AssertionError("5Hz target echo request missing")
        if not any(s.get("target_echo_fresh") for s in node.statuses):
            raise AssertionError("target echo was never observed")
        counts = node.statuses[-1].get("tone_event_counts", {})
        if not all(counts.get(event, 0) > 0 for event in ("FOLLOW_ACTIVE", "LANDING_ACTIVE")):
            raise AssertionError(f"follow/landing tones missing: {counts}")
        if not any(t.msgid == 258 for t in node.tones):
            raise AssertionError("PLAY_TUNE transport missing")
        print("ROS_CH5_CH6_CH8_TAG_RECOVERY_FLOW_OK")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"MODE_FLOW_FAILED:{type(exc).__name__}:{exc}", file=sys.stderr)
        sys.exit(1)
