"""One-shot ROS 2 construction smoke test for the follow tone integration."""

from mavros_msgs.msg import PlayTuneV2
import rclpy

from air_ground_landing.follow_tone_policy import (
    FollowToneEvent,
    FollowTonePolicy,
)
from air_ground_landing_ros2.guided_executor import GuidedExecutor


def main() -> None:
    policy = FollowTonePolicy()
    assert policy.update(
        observe_ready=True,
        follow_active=False,
        landing_active=False,
        exit_confirmed=False,
        now_s=0.0,
    ) == (FollowToneEvent.OBSERVE_READY,)
    assert PlayTuneV2.QBASIC1_1 == 1

    rclpy.init()
    node = GuidedExecutor()
    try:
        assert node.tone_output_enabled is False
        print(
            {
                "node": node.get_name(),
                "tone_format": PlayTuneV2.QBASIC1_1,
                "tone_topic": node.get_parameter("tone_topic").value,
                "target_echo_topic": node.get_parameter("target_echo_topic").value,
            }
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
