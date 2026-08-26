"""ROS 2 adapter for the OV9281 feature-space alignment controller."""

from __future__ import annotations

import json
import time
import urllib.request
from pathlib import Path

import rclpy
from mavros_msgs.msg import PositionTarget
from rclpy.node import Node
from std_msgs.msg import String

from air_ground_landing.hybrid_guidance import IbvsConfig, IbvsFeatureController
from air_ground_landing.landing_target_bridge import BridgeConfig, LandingTargetBridge


class IbvsAdapter(Node):
    """Publish horizontal BODY candidates; never command z, yaw or flight mode."""

    def __init__(self) -> None:
        super().__init__("ibvs_adapter")
        self.declare_parameter("config_path", "")
        self.declare_parameter("status_url", "http://127.0.0.1:8765/api/status")
        self.declare_parameter("candidate_topic", "/landing/ibvs/candidate")
        self.declare_parameter("status_topic", "/landing/ibvs/status")
        self.declare_parameter("poll_rate_hz", 10.0)
        self.declare_parameter("http_timeout_s", 0.15)
        config_path = Path(str(self.get_parameter("config_path").value)).expanduser()
        if not config_path.is_file():
            raise ValueError("config_path must point to moving_landing.prototype.json")
        config = json.loads(config_path.read_text(encoding="utf-8"))
        self.bridge = LandingTargetBridge(BridgeConfig.from_mapping(config))
        self.controller = IbvsFeatureController(IbvsConfig.from_mapping(config))
        self.status_url = str(self.get_parameter("status_url").value)
        self.http_timeout_s = float(self.get_parameter("http_timeout_s").value)
        rate_hz = float(self.get_parameter("poll_rate_hz").value)
        if rate_hz <= 0.0 or self.http_timeout_s <= 0.0:
            raise ValueError("IBVS poll rate and HTTP timeout must be positive")
        self.publisher = self.create_publisher(
            PositionTarget,
            str(self.get_parameter("candidate_topic").value),
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.create_timer(1.0 / rate_hz, self._tick)

    def _tick(self) -> None:
        received_time_s = time.monotonic()
        try:
            with urllib.request.urlopen(self.status_url, timeout=self.http_timeout_s) as response:
                status = json.loads(response.read())
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self._status(False, f"VISION_STATUS_UNAVAILABLE:{type(exc).__name__}")
            return
        bridge_result = self.bridge.process_status(
            status,
            received_time_s=received_time_s,
            wall_time_usec=time.time_ns() // 1000,
        )
        if not bridge_result.accepted:
            self._status(False, f"BRIDGE_REJECTED:{bridge_result.reason}")
            return
        features = self.controller.process_status(
            status,
            bridge_result.observation,
            now_s=received_time_s,
        )
        if not features.valid:
            self._status(False, features.reason)
            return

        target = PositionTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.coordinate_frame = PositionTarget.FRAME_BODY_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE
        )
        # The pure controller outputs aircraft BODY_FRD.  MAVROS expects ROS
        # base_link FLU values and performs FLU->FRD when forwarding BODY_NED.
        body_frd = features.correction_body_frd_mps
        target.velocity.x = float(body_frd[0])
        target.velocity.y = float(-body_frd[1])
        target.velocity.z = 0.0
        self.publisher.publish(target)
        self._status(
            True,
            features.reason,
            tag_id=features.tag_id,
            aligned=features.aligned,
            centroid_error_px=features.centroid_error_px,
        )

    def _status(self, healthy: bool, reason: str, **extra) -> None:
        message = String()
        message.data = json.dumps(
            {"source": "IBVS_ROS2_ADAPTER", "healthy": healthy, "reason": reason, **extra},
            separators=(",", ":"),
        )
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IbvsAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
