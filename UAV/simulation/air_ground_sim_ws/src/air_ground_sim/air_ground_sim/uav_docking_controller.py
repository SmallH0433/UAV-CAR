"""Visual-servo docking controller for stopped and moving Hunter platforms."""

import json
import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .docking_math import (
    body_feedforward_from_ugv,
    inside_capture_envelope,
    propagate_map_pose_with_odometry,
    visual_centering_velocity,
)
from .navigation_math import goal_velocity_body, wrap_angle, yaw_from_quaternion
from .protocol import clamp
from .ros_compat import run_shutdown_action
from .runtime_timing import create_steady_timer


class UavDockingController(Node):
    """Coarse map rendezvous followed by guarded AprilTag visual descent."""

    VALID_MODES = ("stopped", "follow", "moving")

    def __init__(self) -> None:
        super().__init__("uav_docking_controller")
        self.declare_parameter("command_topic", "/uav/docking/cmd_vel")
        self.declare_parameter("mode_topic", "/uav/docking/mode")
        self.declare_parameter("uav_odom_topic", "/uav/odom")
        self.declare_parameter("ugv_pose_topic", "/amcl_pose")
        self.declare_parameter("ugv_odom_topic", "/odometry/filtered")
        self.declare_parameter("apriltag_status_topic", "/apriltag/status")
        self.declare_parameter("perception_status_topic", "/uav/perception/status")
        self.declare_parameter("approach_altitude_m", 2.6)
        self.declare_parameter("follow_altitude_m", 2.8)
        self.declare_parameter("follow_distance_m", 1.6)
        self.declare_parameter("deck_height_m", 0.39)
        self.declare_parameter("capture_altitude_m", 0.72)
        self.declare_parameter("capture_deck_range_m", 0.35)
        self.declare_parameter("max_xy_mps", 0.75)
        self.declare_parameter("max_z_mps", 0.30)
        self.declare_parameter("max_yaw_rate_rps", 0.45)
        self.declare_parameter("coarse_gain", 0.55)
        self.declare_parameter("visual_gain", 0.55)
        self.declare_parameter("vertical_gain", 0.55)
        self.declare_parameter("visual_deadband", 0.035)
        self.declare_parameter("descent_rate_mps", 0.18)
        self.declare_parameter("near_deck_descent_rate_mps", 0.10)
        self.declare_parameter("near_deck_freshness_margin_m", 0.55)
        self.declare_parameter("tag_timeout_s", 2.5)
        self.declare_parameter("pose_timeout_s", 0.8)
        self.declare_parameter("capture_error", 0.11)
        self.declare_parameter("descent_error", 0.16)
        self.declare_parameter("visual_command_freshness_s", 0.45)
        self.declare_parameter("descent_command_freshness_s", 1.0)
        self.declare_parameter("visual_reacquire_hold_s", 8.0)
        self.declare_parameter("minimum_capture_tag_area_px", 1400.0)

        self.approach_altitude = float(
            self.get_parameter("approach_altitude_m").value
        )
        self.follow_altitude = float(self.get_parameter("follow_altitude_m").value)
        self.follow_distance = float(self.get_parameter("follow_distance_m").value)
        self.deck_height = float(self.get_parameter("deck_height_m").value)
        self.capture_altitude = float(self.get_parameter("capture_altitude_m").value)
        self.capture_deck_range = max(
            0.0, float(self.get_parameter("capture_deck_range_m").value)
        )
        self.max_xy = float(self.get_parameter("max_xy_mps").value)
        self.max_z = float(self.get_parameter("max_z_mps").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate_rps").value)
        self.coarse_gain = float(self.get_parameter("coarse_gain").value)
        self.visual_gain = float(self.get_parameter("visual_gain").value)
        self.vertical_gain = float(self.get_parameter("vertical_gain").value)
        self.visual_deadband = float(self.get_parameter("visual_deadband").value)
        self.descent_rate = float(self.get_parameter("descent_rate_mps").value)
        self.near_deck_descent_rate = min(
            self.descent_rate,
            max(0.02, float(self.get_parameter("near_deck_descent_rate_mps").value)),
        )
        self.near_deck_freshness_margin = max(
            0.1, float(self.get_parameter("near_deck_freshness_margin_m").value)
        )
        self.tag_timeout = float(self.get_parameter("tag_timeout_s").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout_s").value)
        self.capture_error = float(self.get_parameter("capture_error").value)
        self.descent_error = max(
            self.capture_error, float(self.get_parameter("descent_error").value)
        )
        self.visual_command_freshness = max(
            0.05, float(self.get_parameter("visual_command_freshness_s").value)
        )
        self.descent_command_freshness = max(
            self.visual_command_freshness,
            float(self.get_parameter("descent_command_freshness_s").value),
        )
        self.visual_reacquire_hold = max(
            self.tag_timeout,
            float(self.get_parameter("visual_reacquire_hold_s").value),
        )
        self.minimum_tag_area = float(
            self.get_parameter("minimum_capture_tag_area_px").value
        )

        self.enabled = False
        self.mode = "stopped"
        self.state = "disabled"
        self.reason = "disabled"
        self.capture_ready = False
        self.uav_odom = None
        self.ugv_pose = None
        self.ugv_odom = None
        self.ugv_map_anchor = None
        self.ugv_odom_anchor = None
        self.apriltag = {}
        self.perception = {}
        self.last_uav = 0.0
        self.last_ugv_pose = 0.0
        self.last_ugv_odom = 0.0
        self.last_tag_status = 0.0
        self.last_visual_seen = 0.0
        self.last_command = Twist()

        self.command_publisher = self.create_publisher(
            Twist, str(self.get_parameter("command_topic").value), 10
        )
        self.status_publisher = self.create_publisher(
            String, "/uav/docking/status", 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("uav_odom_topic").value),
            self.on_uav_odom,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("ugv_pose_topic").value),
            self.on_ugv_pose,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("ugv_odom_topic").value),
            self.on_ugv_odom,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("apriltag_status_topic").value),
            self.on_tag_status,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("perception_status_topic").value),
            self.on_perception_status,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mode_topic").value),
            self.on_mode,
            10,
        )
        self.create_service(SetBool, "~/enable", self.on_enable)
        self.control_timer = create_steady_timer(self, 0.05, self.control_tick)
        self.status_timer = create_steady_timer(self, 0.25, self.publish_status)

    def on_uav_odom(self, message: Odometry) -> None:
        self.uav_odom = message
        self.last_uav = time.monotonic()

    def on_ugv_pose(self, message: PoseWithCovarianceStamped) -> None:
        self.ugv_pose = message
        self.last_ugv_pose = time.monotonic()
        pose = message.pose.pose
        self.ugv_map_anchor = (
            float(pose.position.x),
            float(pose.position.y),
            self._yaw(pose.orientation),
        )
        self.ugv_odom_anchor = self._odom_state(self.ugv_odom)

    def on_ugv_odom(self, message: Odometry) -> None:
        self.ugv_odom = message
        self.last_ugv_odom = time.monotonic()
        if self.ugv_map_anchor is not None and self.ugv_odom_anchor is None:
            self.ugv_odom_anchor = self._odom_state(message)

    def _load_json(self, message: String) -> dict:
        try:
            return json.loads(message.data)
        except json.JSONDecodeError:
            return {}

    def on_tag_status(self, message: String) -> None:
        self.apriltag = self._load_json(message)
        self.last_tag_status = time.monotonic()

    def on_perception_status(self, message: String) -> None:
        self.perception = self._load_json(message)

    def on_mode(self, message: String) -> None:
        requested = message.data.strip().lower()
        if requested in self.VALID_MODES:
            self.mode = requested
            self.capture_ready = False
            self.reason = f"mode_{requested}"
        elif requested == "abort":
            self.enabled = False
            self.stop("operator_abort")
        else:
            self.get_logger().warning(f"Ignored unsupported docking mode '{requested}'")

    def on_enable(self, request: SetBool.Request, response: SetBool.Response):
        self.enabled = bool(request.data)
        self.capture_ready = False
        if not self.enabled:
            self.stop("disabled")
        else:
            self.state = "waiting_inputs"
            self.reason = "enabled"
        response.success = True
        response.message = "Docking enabled" if request.data else "Docking disabled"
        return response

    def stop(self, reason: str) -> None:
        self.state = "disabled" if not self.enabled else "holding"
        self.reason = reason
        self.last_command = Twist()
        self.command_publisher.publish(self.last_command)

    def _yaw(self, orientation) -> float:
        return yaw_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

    def _odom_state(self, odometry):
        if odometry is None:
            return None
        pose = odometry.pose.pose
        return (
            float(pose.position.x),
            float(pose.position.y),
            self._yaw(pose.orientation),
        )

    def _ugv_map_state(self):
        current_odom = self._odom_state(self.ugv_odom)
        if (
            self.ugv_map_anchor is None
            or self.ugv_odom_anchor is None
            or current_odom is None
        ):
            return self.ugv_map_anchor
        return propagate_map_pose_with_odometry(
            self.ugv_map_anchor,
            self.ugv_odom_anchor,
            current_odom,
        )

    def _inputs_fresh(self, now: float) -> bool:
        return (
            self.uav_odom is not None
            and self._ugv_map_state() is not None
            and self.ugv_odom is not None
            and now - self.last_uav <= self.pose_timeout
            and now - self.last_ugv_odom <= self.pose_timeout
        )

    def _tag_visible(self, now: float) -> bool:
        return (
            now - self.last_tag_status <= self.tag_timeout
            and int(self.apriltag.get("detections", 0)) > 0
            and self._visual_detection_age(now) <= self.tag_timeout
        )

    def _visual_detection_age(self, now: float) -> float:
        reported = self.apriltag.get("detection_age_s")
        if reported is None or self.last_tag_status == 0.0:
            return float("inf")
        return max(0.0, float(reported)) + max(0.0, now - self.last_tag_status)

    def _ugv_feedforward(self, uav_yaw: float, ugv_yaw: float) -> tuple:
        if (
            self.mode not in ("follow", "moving")
            or self.ugv_odom is None
            or time.monotonic() - self.last_ugv_odom > self.pose_timeout
        ):
            return (0.0, 0.0)
        return body_feedforward_from_ugv(
            self.ugv_odom.twist.twist.linear.x, ugv_yaw, uav_yaw
        )

    def _healthy_deck_range(self):
        sensors = self.perception.get("sensors") or {}
        down_sensor = sensors.get("ultrasonic_down") or {}
        if not bool(down_sensor.get("healthy", False)):
            return None
        ranges = self.perception.get("ultrasonic_ranges_m") or {}
        value = ranges.get("down")
        try:
            measured = float(value)
        except (TypeError, ValueError):
            return None
        return measured if math.isfinite(measured) and measured >= 0.0 else None

    def _apply_obstacle_gate(self, command: Twist, terminal_visual: bool) -> None:
        sectors = self.perception.get("sectors_m") or {}
        threshold = 0.65
        for name, value in sectors.items():
            if value is None or float(value) > threshold:
                continue
            if name == "front" and command.linear.x > 0.0 and not terminal_visual:
                command.linear.x = 0.0
            elif name == "rear" and command.linear.x < 0.0:
                command.linear.x = 0.0
            elif name == "left" and command.linear.y > 0.0:
                command.linear.y = 0.0
            elif name == "right" and command.linear.y < 0.0:
                command.linear.y = 0.0
            elif name == "up" and command.linear.z > 0.0:
                command.linear.z = 0.0
            elif name == "down" and command.linear.z < 0.0 and not terminal_visual:
                command.linear.z = 0.0

    def control_tick(self) -> None:
        now = time.monotonic()
        if not self.enabled:
            self.stop("disabled")
            return
        if not self._inputs_fresh(now):
            self.stop("pose_timeout")
            return

        uav_position = self.uav_odom.pose.pose.position
        uav_yaw = self._yaw(self.uav_odom.pose.pose.orientation)
        ugv_x, ugv_y, ugv_yaw = self._ugv_map_state()
        target_x = ugv_x
        target_y = ugv_y
        target_altitude = self.approach_altitude
        if self.mode == "follow":
            target_x -= self.follow_distance * math.cos(ugv_yaw)
            target_y -= self.follow_distance * math.sin(ugv_yaw)
            target_altitude = self.follow_altitude

        dx = target_x - uav_position.x
        dy = target_y - uav_position.y
        horizontal_distance = math.hypot(dx, dy)
        command = Twist()
        coarse = goal_velocity_body(dx, dy, uav_yaw, self.coarse_gain, self.max_xy)
        feed_forward = self._ugv_feedforward(uav_yaw, ugv_yaw)
        tag_visible = self._tag_visible(now)
        if tag_visible:
            self.last_visual_seen = now
        terminal_mode = self.mode in ("stopped", "moving")
        terminal_visual = terminal_mode and tag_visible and horizontal_distance < 1.2
        visual_recent = (
            self.last_visual_seen > 0.0
            and now - self.last_visual_seen <= self.visual_reacquire_hold
        )
        visual_reacquire = (
            terminal_mode
            and not tag_visible
            and visual_recent
            and horizontal_distance < 1.2
        )

        if self.mode == "follow" or (not terminal_visual and not visual_reacquire):
            command.linear.x = coarse.forward + feed_forward[0]
            command.linear.y = coarse.left + feed_forward[1]
            command.linear.z = clamp(
                self.vertical_gain * (target_altitude - uav_position.z),
                -self.max_z,
                self.max_z,
            )
            self.state = "following" if self.mode == "follow" else "coarse_rendezvous"
            self.reason = "map_relative_control"
            self.capture_ready = False
        elif visual_reacquire:
            # A camera frame gap near the deck must not command a climb. Keep
            # the map/odometry relative position and wait for visual lock to
            # return. A sustained loss still expires into the coarse branch.
            command.linear.x = coarse.forward + feed_forward[0]
            command.linear.y = coarse.left + feed_forward[1]
            command.linear.z = 0.0
            self.capture_ready = False
            self.state = "visual_reacquire_hold"
            self.reason = "holding_altitude_for_tag_reacquisition"
        else:
            error_x = float(self.apriltag.get("error_x", 0.0))
            error_y = float(self.apriltag.get("error_y", 0.0))
            visual_age = self._visual_detection_age(now)
            fresh_visual = visual_age <= self.visual_command_freshness
            near_deck = (
                uav_position.z
                <= self.capture_altitude + self.near_deck_freshness_margin
            )
            descent_freshness = (
                self.visual_command_freshness
                if near_deck
                else self.descent_command_freshness
            )
            fresh_descent = visual_age <= descent_freshness
            visual_forward, visual_left = visual_centering_velocity(
                error_x,
                error_y,
                self.visual_gain,
                self.max_xy * 0.65,
                self.visual_deadband,
            )
            command.linear.x = feed_forward[0]
            command.linear.y = feed_forward[1]
            # Descend only through a tighter visual gate than the coarse tag
            # acquisition envelope.  Otherwise a small lateral error grows in
            # pixels near the deck, crops the tag, and produces a bounce-back
            # cycle before the capture envelope can be satisfied.
            centered = math.hypot(error_x, error_y) <= self.descent_error
            if not fresh_visual and not (centered and fresh_descent):
                command.linear.z = 0.0
                self.state = "visual_frame_hold"
                self.reason = "awaiting_fresh_tag_frame"
            elif centered:
                if fresh_visual:
                    command.linear.x += visual_forward
                    command.linear.y += visual_left
                command.linear.z = -(
                    self.near_deck_descent_rate if near_deck else self.descent_rate
                )
                self.state = "visual_descent_moving" if self.mode == "moving" else "visual_descent_stopped"
                self.reason = "tag_centered_descending"
            else:
                command.linear.x += visual_forward
                command.linear.y += visual_left
                # Hold altitude while centering. Climbing all the way back to
                # approach altitude couples image error into a limit cycle:
                # the target shrinks while ascending, then grows again while
                # descending. A stationary vertical command lets the visual
                # loop finish lateral alignment before descent resumes.
                command.linear.z = 0.0
                self.state = "visual_centering"
                self.reason = "centering_before_descent"

            self.capture_ready = inside_capture_envelope(
                tag_visible and fresh_visual,
                error_x,
                error_y,
                uav_position.z,
                self.capture_altitude,
                self.capture_error,
                float(self.apriltag.get("tag_area_px", 0.0)),
                self.minimum_tag_area,
                self._healthy_deck_range(),
                self.capture_deck_range,
            )
            if self.capture_ready:
                command.linear.x = feed_forward[0]
                command.linear.y = feed_forward[1]
                command.linear.z = 0.0
                self.state = "capture_ready"
                self.reason = "inside_latch_envelope"

        magnitude = math.hypot(command.linear.x, command.linear.y)
        if magnitude > self.max_xy:
            scale = self.max_xy / magnitude
            command.linear.x *= scale
            command.linear.y *= scale
        command.linear.z = clamp(command.linear.z, -self.max_z, self.max_z)
        command.angular.z = clamp(
            0.8 * wrap_angle(ugv_yaw - uav_yaw),
            -self.max_yaw_rate,
            self.max_yaw_rate,
        )
        self._apply_obstacle_gate(command, terminal_visual)
        self.last_command = command
        self.command_publisher.publish(command)

    def publish_status(self) -> None:
        now = time.monotonic()
        uav_position = self.uav_odom.pose.pose.position if self.uav_odom else None
        ugv_state = self._ugv_map_state()
        separation = None
        if uav_position is not None and ugv_state is not None:
            separation = math.sqrt(
                (uav_position.x - ugv_state[0]) ** 2
                + (uav_position.y - ugv_state[1]) ** 2
                + (uav_position.z - self.deck_height) ** 2
            )
        message = String()
        message.data = json.dumps(
            {
                "active": self.enabled,
                "mode": self.mode,
                "state": self.state,
                "reason": self.reason,
                "capture_ready": self.capture_ready,
                "tag_visible": self._tag_visible(now),
                "visual_detection_age_s": None
                if not math.isfinite(self._visual_detection_age(now))
                else round(self._visual_detection_age(now), 3),
                "descent_frame_fresh": self._visual_detection_age(now)
                <= self.descent_command_freshness,
                "visual_reacquire_age_s": None
                if self.last_visual_seen == 0.0
                else round(now - self.last_visual_seen, 3),
                "uav_odom_age_s": None
                if self.last_uav == 0.0
                else round(now - self.last_uav, 3),
                "ugv_odom_age_s": None
                if self.last_ugv_odom == 0.0
                else round(now - self.last_ugv_odom, 3),
                "map_anchor_age_s": None
                if self.last_ugv_pose == 0.0
                else round(now - self.last_ugv_pose, 3),
                "separation_m": None if separation is None else round(separation, 3),
                "deck_range_m": self._healthy_deck_range(),
                "capture_deck_range_m": self.capture_deck_range,
                "uav_position": None if uav_position is None else [
                    round(uav_position.x, 3), round(uav_position.y, 3), round(uav_position.z, 3)
                ],
                "ugv_position": None if ugv_state is None else [
                    round(ugv_state[0], 3), round(ugv_state[1], 3)
                ],
                "command_body": [
                    round(self.last_command.linear.x, 3),
                    round(self.last_command.linear.y, 3),
                    round(self.last_command.linear.z, 3),
                    round(self.last_command.angular.z, 3),
                ],
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UavDockingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            run_shutdown_action(lambda: node.stop("shutdown"))
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
