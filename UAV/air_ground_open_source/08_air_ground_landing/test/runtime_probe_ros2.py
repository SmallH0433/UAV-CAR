"""Read-only live probe for MAVROS and guided-executor state."""

import json
import time

from mavros_msgs.msg import RCIn, State
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class RuntimeProbe(Node):
    def __init__(self) -> None:
        super().__init__("follow_runtime_probe")
        self.guided_status = None
        self.mavros_state = None
        self.rc_channels = None
        self.create_subscription(
            String,
            "/landing/guided_executor/status",
            self._guided_status,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            State,
            "/mavros/state",
            self._mavros_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            RCIn,
            "/mavros/rc/in",
            self._rc,
            qos_profile_sensor_data,
        )

    def _guided_status(self, message: String) -> None:
        try:
            self.guided_status = json.loads(message.data)
        except json.JSONDecodeError:
            self.guided_status = {"raw": message.data}

    def _mavros_state(self, message: State) -> None:
        self.mavros_state = {
            "connected": bool(message.connected),
            "armed": bool(message.armed),
            "mode": message.mode,
        }

    def _rc(self, message: RCIn) -> None:
        self.rc_channels = list(message.channels)


def main() -> int:
    rclpy.init()
    node = RuntimeProbe()
    deadline = time.monotonic() + 10.0
    try:
        while time.monotonic() < deadline and node.guided_status is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
        payload = {
            "guided_executor": node.guided_status,
            "mavros_state": node.mavros_state,
            "rc_channels": node.rc_channels,
            "tone_subscribers": [
                {
                    "node": f"{info.node_namespace.rstrip('/')}/{info.node_name}",
                    "type": info.topic_type,
                }
                for info in node.get_subscriptions_info_by_topic("/mavros/play_tune")
            ],
            "target_echo_publishers": [
                {
                    "node": f"{info.node_namespace.rstrip('/')}/{info.node_name}",
                    "type": info.topic_type,
                    "reliability": str(info.qos_profile.reliability),
                }
                for info in node.get_publishers_info_by_topic(
                    "/mavros/setpoint_raw/target_local"
                )
            ],
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0 if node.guided_status is not None else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
