"""Service-gated 3D waypoint navigation with horizontal lidar avoidance."""

import json
import math
import time

from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .airspace import AirspaceRules
from .local_planner import select_body_velocity
from .navigation_math import (
    apply_lidar_avoidance,
    goal_velocity_body,
    minimum_valid_range,
    vertical_goal_velocity,
    wrap_angle,
    yaw_from_quaternion,
)
from .protocol import clamp
from .ros_compat import run_shutdown_action
from .runtime_timing import create_steady_timer


class UavNavigation(Node):
    """Navigate in ArduPilot local coordinates while preserving pilot authority."""

    def __init__(self) -> None:
        super().__init__("uav_navigation")
        self.declare_parameter("odom_topic", "/uav/odom")
        self.declare_parameter("scan_topic", "/uav/scan")
        self.declare_parameter("goal_topic", "/uav/nav/goal")
        self.declare_parameter("command_topic", "/uav/nav/cmd_vel")
        self.declare_parameter("telemetry_topic", "/uav/mavlink/status")
        self.declare_parameter(
            "perception_vector_topic", "/uav/perception/avoidance_vector"
        )
        self.declare_parameter("perception_status_topic", "/uav/perception/status")
        self.declare_parameter("command_enabled", False)
        self.declare_parameter("require_flight_ready", True)
        self.declare_parameter("require_perception", True)
        self.declare_parameter("odom_timeout_s", 0.5)
        self.declare_parameter("scan_timeout_s", 0.5)
        self.declare_parameter("perception_timeout_s", 0.8)
        self.declare_parameter("xy_gain", 0.6)
        self.declare_parameter("z_gain", 0.6)
        self.declare_parameter("yaw_gain", 1.0)
        self.declare_parameter("max_xy_mps", 1.0)
        self.declare_parameter("max_z_mps", 0.5)
        self.declare_parameter("max_yaw_rate_rps", 0.6)
        self.declare_parameter("xy_tolerance_m", 0.35)
        self.declare_parameter("z_tolerance_m", 0.25)
        self.declare_parameter("minimum_navigation_altitude_m", 0.8)
        self.declare_parameter("maximum_altitude_m", 12.0)
        self.declare_parameter("geofence_radius_m", 20.0)
        self.declare_parameter("obstacle_influence_distance_m", 3.0)
        self.declare_parameter("obstacle_hard_stop_distance_m", 1.0)
        self.declare_parameter("obstacle_repulsion_gain", 1.2)
        self.declare_parameter("airspace_prediction_horizon_s", 1.8)
        self.declare_parameter("airspace_margin_m", 0.20)
        self.declare_parameter("no_fly_zones_json", "[]")
        self.declare_parameter("height_limit_zones_json", "[]")

        self.command_enabled = bool(self.get_parameter("command_enabled").value)
        self.require_flight_ready = bool(self.get_parameter("require_flight_ready").value)
        self.require_perception = bool(self.get_parameter("require_perception").value)
        self.odom_timeout = float(self.get_parameter("odom_timeout_s").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout_s").value)
        self.perception_timeout = float(
            self.get_parameter("perception_timeout_s").value
        )
        self.xy_gain = float(self.get_parameter("xy_gain").value)
        self.z_gain = float(self.get_parameter("z_gain").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.max_xy = float(self.get_parameter("max_xy_mps").value)
        self.max_z = float(self.get_parameter("max_z_mps").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate_rps").value)
        self.xy_tolerance = float(self.get_parameter("xy_tolerance_m").value)
        self.z_tolerance = float(self.get_parameter("z_tolerance_m").value)
        self.min_altitude = float(
            self.get_parameter("minimum_navigation_altitude_m").value
        )
        self.max_altitude = float(self.get_parameter("maximum_altitude_m").value)
        self.geofence_radius = float(self.get_parameter("geofence_radius_m").value)
        self.obstacle_influence = float(
            self.get_parameter("obstacle_influence_distance_m").value
        )
        self.obstacle_hard_stop = float(
            self.get_parameter("obstacle_hard_stop_distance_m").value
        )
        self.obstacle_repulsion = float(
            self.get_parameter("obstacle_repulsion_gain").value
        )
        self.airspace_horizon = float(
            self.get_parameter("airspace_prediction_horizon_s").value
        )
        self.airspace_margin = float(self.get_parameter("airspace_margin_m").value)
        try:
            self.airspace = AirspaceRules.from_json(
                self.geofence_radius,
                self.min_altitude,
                self.max_altitude,
                str(self.get_parameter("no_fly_zones_json").value),
                str(self.get_parameter("height_limit_zones_json").value),
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            raise RuntimeError(f"Invalid UAV airspace parameters: {error}") from error

        self.goal = None
        self.odom = None
        self.scan = None
        self.telemetry = {}
        self.perception_status = {}
        self.perception_vector = (0.0, 0.0, 0.0)
        self.last_odom_time = 0.0
        self.last_scan_time = 0.0
        self.last_perception_status_time = 0.0
        self.last_perception_vector_time = 0.0
        self.active = False
        self.goal_reached = False
        self.reason = "disabled" if not self.command_enabled else "waiting_goal"
        self.last_command = Twist()

        self.command_publisher = self.create_publisher(
            Twist, str(self.get_parameter("command_topic").value), 10
        )
        self.status_publisher = self.create_publisher(String, "/uav/navigation/status", 10)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            self.on_goal,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("telemetry_topic").value),
            self.on_telemetry,
            10,
        )
        self.create_subscription(
            Vector3Stamped,
            str(self.get_parameter("perception_vector_topic").value),
            self.on_perception_vector,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("perception_status_topic").value),
            self.on_perception_status,
            10,
        )
        self.enable_service = self.create_service(SetBool, "~/enable", self.on_enable)
        self.control_timer = create_steady_timer(self, 0.05, self.control_tick)
        self.status_timer = create_steady_timer(self, 0.5, self.publish_status)

    def on_odom(self, message: Odometry) -> None:
        self.odom = message
        self.last_odom_time = time.monotonic()

    def on_scan(self, message: LaserScan) -> None:
        self.scan = message
        self.last_scan_time = time.monotonic()

    def on_goal(self, message: PoseStamped) -> None:
        frame = message.header.frame_id
        if frame not in ("", "map", "uav_odom"):
            self.reason = f"unsupported_goal_frame_{frame}"
            self.get_logger().error(
                f"UAV goal frame must be map or uav_odom, received '{frame}'"
            )
            return
        position = message.pose.position
        airspace = self.airspace.check(position.x, position.y, position.z)
        if not airspace.allowed:
            self.reason = f"goal_rejected_{airspace.reason}"
            self.get_logger().error(
                f"Rejected UAV goal: {airspace.reason}"
                + (f" ({airspace.zone})" if airspace.zone else "")
            )
            return
        self.goal = message
        self.goal_reached = False
        self.reason = "goal_received"

    def on_telemetry(self, message: String) -> None:
        try:
            self.telemetry = json.loads(message.data)
        except json.JSONDecodeError:
            pass

    def on_perception_vector(self, message: Vector3Stamped) -> None:
        self.perception_vector = (
            float(message.vector.x),
            float(message.vector.y),
            float(message.vector.z),
        )
        self.last_perception_vector_time = time.monotonic()

    def on_perception_status(self, message: String) -> None:
        try:
            self.perception_status = json.loads(message.data)
            self.last_perception_status_time = time.monotonic()
        except json.JSONDecodeError:
            pass

    def perception_ready(self, now: float) -> bool:
        if not self.require_perception:
            return True
        return (
            now - self.last_perception_vector_time <= self.perception_timeout
            and now - self.last_perception_status_time <= self.perception_timeout
            and bool(self.perception_status.get("healthy", False))
        )

    def on_enable(self, request: SetBool.Request, response: SetBool.Response):
        self.command_enabled = bool(request.data)
        if not self.command_enabled:
            self.stop("disabled")
        else:
            self.reason = "waiting_goal" if self.goal is None else "waiting_flight_ready"
        response.success = True
        response.message = (
            "UAV navigation enabled; aircraft must already be armed in GUIDED"
            if request.data
            else "UAV navigation disabled and command zeroed"
        )
        return response

    def flight_ready(self) -> bool:
        if not self.require_flight_ready:
            return True
        return (
            bool(self.telemetry.get("connected", False))
            and bool(self.telemetry.get("armed", False))
            and str(self.telemetry.get("mode", "UNKNOWN")) == "GUIDED"
        )

    def stop(self, reason: str) -> None:
        self.active = False
        self.reason = reason
        self.last_command = Twist()
        self.command_publisher.publish(self.last_command)

    def control_tick(self) -> None:
        now = time.monotonic()
        if not self.command_enabled:
            self.stop("disabled")
            return
        if self.goal is None:
            self.stop("waiting_goal")
            return
        if not self.flight_ready():
            self.stop("waiting_armed_guided")
            return
        if self.odom is None or now - self.last_odom_time > self.odom_timeout:
            self.stop("odometry_timeout")
            return
        if self.scan is None or now - self.last_scan_time > self.scan_timeout:
            self.stop("lidar_timeout")
            return
        if not self.perception_ready(now):
            self.stop("perception_unhealthy_or_timeout")
            return

        position = self.odom.pose.pose.position
        current_airspace = self.airspace.check(position.x, position.y, position.z)
        if not current_airspace.allowed:
            self.stop(f"airspace_violation_{current_airspace.reason}")
            return

        goal = self.goal.pose.position
        dx = goal.x - position.x
        dy = goal.y - position.y
        dz = goal.z - position.z
        planar_distance = math.hypot(dx, dy)
        if planar_distance <= self.xy_tolerance and abs(dz) <= self.z_tolerance:
            self.goal_reached = True
            self.stop("goal_reached")
            return

        orientation = self.odom.pose.pose.orientation
        yaw = yaw_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        desired = goal_velocity_body(dx, dy, yaw, self.xy_gain, self.max_xy)
        desired_vertical = vertical_goal_velocity(dz, self.z_gain, self.max_z)

        perception_fresh = (
            now - self.last_perception_vector_time <= self.perception_timeout
            and now - self.last_perception_status_time <= self.perception_timeout
        )
        if perception_fresh:
            raw_sectors = self.perception_status.get("sectors_m") or {}
            sectors = {
                name: math.inf if raw_sectors.get(name) is None else float(raw_sectors[name])
                for name in ("front", "rear", "left", "right", "up", "down")
            }

            def candidate_is_safe(candidate) -> bool:
                body_x, body_y, body_z = candidate
                body_z = clamp(body_z, -self.max_z, self.max_z)
                world_x = math.cos(yaw) * body_x - math.sin(yaw) * body_y
                world_y = math.sin(yaw) * body_x + math.cos(yaw) * body_y
                start = (position.x, position.y, position.z)
                end = (
                    position.x + world_x * self.airspace_horizon,
                    position.y + world_y * self.airspace_horizon,
                    position.z + body_z * self.airspace_horizon,
                )
                return self.airspace.segment_allowed(
                    start, end, margin=self.airspace_margin, samples=5
                ).allowed

            selected = select_body_velocity(
                (desired.forward, desired.left, desired_vertical),
                self.perception_vector,
                sectors,
                self.obstacle_hard_stop,
                self.obstacle_influence,
                self.max_xy,
                self.obstacle_repulsion,
                candidate_is_safe,
            )
            avoided_forward, avoided_left, selected_vertical = selected
            if sectors["up"] <= self.obstacle_hard_stop and selected_vertical > 0.0:
                selected_vertical = 0.0
            if sectors["down"] <= self.obstacle_hard_stop and selected_vertical < 0.0:
                selected_vertical = 0.0
        else:
            # Compatibility path for a deliberately configured 2D-only system.
            fallback = apply_lidar_avoidance(
                desired,
                self.scan.ranges,
                self.scan.angle_min,
                self.scan.angle_increment,
                self.obstacle_influence,
                self.obstacle_hard_stop,
                self.obstacle_repulsion,
                self.max_xy,
            )
            avoided_forward, avoided_left = fallback.forward, fallback.left
            selected_vertical = desired_vertical
        desired_yaw = math.atan2(dy, dx) if planar_distance > self.xy_tolerance else yaw

        command = Twist()
        command.linear.x = avoided_forward
        command.linear.y = avoided_left
        command.linear.z = clamp(selected_vertical, -self.max_z, self.max_z)
        command.angular.z = clamp(
            self.yaw_gain * wrap_angle(desired_yaw - yaw),
            -self.max_yaw_rate,
            self.max_yaw_rate,
        )
        self.last_command = command
        self.active = True
        if (
            planar_distance > self.xy_tolerance
            and math.hypot(command.linear.x, command.linear.y) < 0.02
        ):
            self.reason = "local_planner_blocked_hovering"
        else:
            self.reason = (
                "navigating_with_3d_fused_avoidance"
                if perception_fresh
                else "navigating_with_2d_fallback"
            )
        self.command_publisher.publish(command)

    def publish_status(self) -> None:
        position = self.odom.pose.pose.position if self.odom is not None else None
        goal = self.goal.pose.position if self.goal is not None else None
        status = {
            "active": self.active,
            "enabled": self.command_enabled,
            "reason": self.reason,
            "goal_reached": self.goal_reached,
            "flight_ready": self.flight_ready(),
            "perception_ready": self.perception_ready(time.monotonic()),
            "perception_required": self.require_perception,
            "position": None
            if position is None
            else [round(position.x, 3), round(position.y, 3), round(position.z, 3)],
            "goal": None
            if goal is None
            else [round(goal.x, 3), round(goal.y, 3), round(goal.z, 3)],
            "minimum_lidar_range_m": None
            if self.scan is None or not math.isfinite(minimum_valid_range(self.scan.ranges))
            else round(minimum_valid_range(self.scan.ranges), 3),
            "minimum_fused_obstacle_m": self.perception_status.get(
                "minimum_obstacle_m"
            ),
            "airspace": {
                "geofence_radius_m": self.geofence_radius,
                "minimum_altitude_m": self.min_altitude,
                "maximum_altitude_m": self.max_altitude,
                "no_fly_zone_count": len(self.airspace.no_fly_zones),
                "height_limit_zone_count": len(self.airspace.height_limit_zones),
            },
            "command_body": [
                round(self.last_command.linear.x, 3),
                round(self.last_command.linear.y, 3),
                round(self.last_command.linear.z, 3),
                round(self.last_command.angular.z, 3),
            ],
        }
        message = String()
        message.data = json.dumps(status, ensure_ascii=False, sort_keys=True)
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UavNavigation()
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
