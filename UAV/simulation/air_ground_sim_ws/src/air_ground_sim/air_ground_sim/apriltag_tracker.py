"""AprilTag ID 0 detector and guarded body-velocity tracking controller."""

import json
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .protocol import tracking_velocity_from_image
from .ros_compat import run_shutdown_action
from .runtime_timing import create_steady_timer


class AprilTagTracker(Node):
    """Track a downward-facing tag only after an explicit RC/service request."""

    def __init__(self) -> None:
        super().__init__("apriltag_tracker")
        self.declare_parameter("image_topic", "/vision/image_raw")
        # Safe standalone default: enter the UAV authority mux instead of
        # writing directly to the MAVLink bridge command topic.
        self.declare_parameter("command_topic", "/uav/follow/cmd_vel")
        self.declare_parameter("telemetry_topic", "/uav/mavlink/status")
        self.declare_parameter("debug_image_topic", "/apriltag/debug_image")
        self.declare_parameter("tag_id", 0)
        self.declare_parameter("rc_channel", 7)
        self.declare_parameter("rc_enable_pwm", 1700)
        self.declare_parameter("allow_service_enable", False)
        self.declare_parameter("min_altitude_m", 1.0)
        self.declare_parameter("gain", 0.8)
        self.declare_parameter("deadband", 0.06)
        self.declare_parameter("max_xy_mps", 0.6)
        self.declare_parameter("detection_timeout_s", 0.45)
        self.declare_parameter("tag_lost_abort_s", 2.0)

        self.tag_id = int(self.get_parameter("tag_id").value)
        self.rc_channel = int(self.get_parameter("rc_channel").value)
        self.rc_enable_pwm = int(self.get_parameter("rc_enable_pwm").value)
        self.allow_service_enable = bool(self.get_parameter("allow_service_enable").value)
        self.min_altitude = float(self.get_parameter("min_altitude_m").value)
        self.gain = float(self.get_parameter("gain").value)
        self.deadband = float(self.get_parameter("deadband").value)
        self.max_xy = float(self.get_parameter("max_xy_mps").value)
        self.detection_timeout = float(self.get_parameter("detection_timeout_s").value)
        self.tag_lost_abort = float(self.get_parameter("tag_lost_abort_s").value)

        self.cv_bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.detector_parameters = cv2.aruco.DetectorParameters_create()

        image_topic = str(self.get_parameter("image_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)
        telemetry_topic = str(self.get_parameter("telemetry_topic").value)
        debug_topic = str(self.get_parameter("debug_image_topic").value)

        self.command_publisher = self.create_publisher(Twist, command_topic, 10)
        self.debug_publisher = self.create_publisher(Image, debug_topic, 2)
        self.status_publisher = self.create_publisher(String, "/apriltag/status", 10)
        self.image_subscription = self.create_subscription(Image, image_topic, self.on_image, 10)
        self.telemetry_subscription = self.create_subscription(
            String, telemetry_topic, self.on_telemetry, 10
        )
        self.enable_service = self.create_service(SetBool, "~/enable", self.on_enable)
        self.guided_client = self.create_client(
            SetBool, "/uav_mavlink_bridge/guided_mode"
        )
        self.control_timer = create_steady_timer(self, 0.1, self.control_tick)
        self.status_timer = create_steady_timer(self, 0.5, self.publish_status)

        self.service_requested = False
        self.active = False
        self.fault_latched = False
        self.reason = "standby"
        self.telemetry = {}
        self.rc_pwm = None
        self.last_image = 0.0
        self.last_detection = 0.0
        self.active_since = 0.0
        self.tag_visible = False
        self.error_x = 0.0
        self.error_y = 0.0
        self.tag_area_px = 0.0
        self.frames = 0
        self.detections = 0
        self.last_command = Twist()
        self.get_logger().info(
            f"AprilTag tracker waiting for tag36h11 ID {self.tag_id}; "
            f"RC{self.rc_channel} >= {self.rc_enable_pwm} enables takeover"
        )

    def on_enable(self, request: SetBool.Request, response: SetBool.Response):
        if request.data and not self.allow_service_enable:
            response.success = False
            response.message = "Service takeover is disabled in this profile; use the RC switch"
            return response
        self.service_requested = bool(request.data)
        if not request.data:
            self.fault_latched = False
        response.success = True
        response.message = (
            "Tracking takeover requested" if request.data else "Tracking takeover released"
        )
        return response

    def on_telemetry(self, message: String) -> None:
        try:
            self.telemetry = json.loads(message.data)
        except json.JSONDecodeError:
            return
        channels = self.telemetry.get("rc_channels") or {}
        value = channels.get(str(self.rc_channel))
        self.rc_pwm = int(value) if value is not None else None

    def on_image(self, message: Image) -> None:
        now = time.monotonic()
        self.last_image = now
        self.frames += 1
        debug_requested = self.debug_publisher.get_subscription_count() > 0
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:  # cv_bridge reports the exact conversion problem
            self.reason = f"image_conversion_error: {error}"
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_parameters
        )
        self.tag_visible = False
        if ids is not None:
            for index, detected_id in enumerate(ids.flatten()):
                if int(detected_id) != self.tag_id:
                    continue
                points = corners[index].reshape(4, 2)
                center = points.mean(axis=0)
                height, width = gray.shape
                self.error_x = float((center[0] - width / 2.0) / (width / 2.0))
                self.error_y = float((center[1] - height / 2.0) / (height / 2.0))
                self.tag_area_px = float(abs(cv2.contourArea(points.astype("float32"))))
                self.tag_visible = True
                self.last_detection = now
                self.detections += 1
                if debug_requested:
                    cv2.polylines(frame, [points.astype("int32")], True, (0, 255, 0), 3)
                    cv2.circle(frame, tuple(center.astype("int32")), 7, (0, 0, 255), -1)
                    cv2.putText(
                        frame,
                        f"tag36h11 ID {self.tag_id}",
                        tuple(points[0].astype("int32")),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0),
                        2,
                    )
                break

        if debug_requested:
            state_text = "TRACKING" if self.active else "STANDBY"
            color = (0, 255, 0) if self.active else (0, 200, 255)
            cv2.putText(
                frame,
                state_text,
                (20, 38),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                color,
                2,
            )
            debug = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            debug.header = message.header
            self.debug_publisher.publish(debug)

    def control_tick(self) -> None:
        now = time.monotonic()
        rc_requested = self.rc_pwm is not None and self.rc_pwm >= self.rc_enable_pwm
        raw_requested = bool(rc_requested or self.service_requested)
        if not raw_requested:
            self.fault_latched = False
            if self.active:
                self.deactivate("operator_released")
            else:
                self.reason = "standby"
            return
        if self.fault_latched:
            self.reason = "fault_latched_release_switch"
            return

        armed = bool(self.telemetry.get("armed", False))
        altitude = self.telemetry.get("relative_alt_m")
        altitude = float(altitude) if altitude is not None else 0.0
        mode = str(self.telemetry.get("mode", "UNKNOWN"))

        if not self.active:
            if not armed:
                self.reason = "waiting_aircraft_armed"
                return
            if altitude < self.min_altitude:
                self.reason = f"waiting_altitude_{altitude:.2f}m"
                return
            self.active = True
            self.active_since = now
            self.reason = "requesting_guided"
            self.request_guided(True)

        if not armed:
            self.abort("aircraft_disarmed")
            return
        if mode not in ("GUIDED", "UNKNOWN") and now - self.active_since > 1.5:
            self.abort("manual_mode_override")
            return

        detection_age = now - max(self.last_detection, self.active_since)
        if detection_age > self.tag_lost_abort:
            self.abort("tag_lost")
            return
        if mode != "GUIDED" or now - self.last_detection > self.detection_timeout:
            self.publish_zero()
            self.reason = "waiting_guided_or_tag"
            return

        velocity = tracking_velocity_from_image(
            self.error_x, self.error_y, self.gain, self.deadband, self.max_xy
        )
        command = Twist()
        command.linear.x = velocity.forward
        command.linear.y = velocity.left
        self.last_command = command
        self.command_publisher.publish(command)
        self.reason = "tracking"

    def deactivate(self, reason: str) -> None:
        """Stop tracker output without changing the pilot-selected flight mode.

        Sending a mode change here is unsafe: a visual fault must not silently
        change the aircraft's throttle semantics. Zero body velocity keeps a
        GUIDED aircraft hovering, while an operator mode switch still takes
        immediate priority in the flight controller.
        """
        self.active = False
        self.reason = reason
        self.publish_zero()

    def abort(self, reason: str) -> None:
        self.fault_latched = True
        self.service_requested = False
        self.deactivate(reason)

    def request_guided(self, enabled: bool) -> None:
        if not self.guided_client.service_is_ready():
            self.get_logger().warning("MAVLink mode service is not ready")
            return
        request = SetBool.Request()
        request.data = bool(enabled)
        self.guided_client.call_async(request)

    def publish_zero(self) -> None:
        self.last_command = Twist()
        self.command_publisher.publish(self.last_command)

    def publish_status(self) -> None:
        now = time.monotonic()
        status = {
            "active": self.active,
            "reason": self.reason,
            "fault_latched": self.fault_latched,
            "service_requested": self.service_requested,
            "rc_channel": self.rc_channel,
            "rc_pwm": self.rc_pwm,
            "tag_family": "tag36h11",
            "tag_id": self.tag_id,
            "tag_visible": self.tag_visible and now - self.last_detection < self.detection_timeout,
            "detection_age_s": None
            if self.last_detection == 0.0
            else round(now - self.last_detection, 3),
            "error_x": round(self.error_x, 4),
            "error_y": round(self.error_y, 4),
            "tag_area_px": round(self.tag_area_px, 1),
            "image_age_s": None if self.last_image == 0.0 else round(now - self.last_image, 3),
            "frames": self.frames,
            "detections": self.detections,
            "command_forward_mps": round(self.last_command.linear.x, 3),
            "command_left_mps": round(self.last_command.linear.y, 3),
        }
        message = String()
        message.data = json.dumps(status, ensure_ascii=False, sort_keys=True)
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AprilTagTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            run_shutdown_action(node.publish_zero)
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
