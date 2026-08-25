from air_ground_sim.web_gateway import (
    CAMERA_TOPICS,
    camera_frame_is_fresh,
    camera_qos_profile,
    requested_camera_keys,
)
from rclpy.qos import HistoryPolicy, ReliabilityPolicy


def test_connected_dashboard_requests_all_camera_streams():
    assert requested_camera_keys(1, {}, 10.0) == set(CAMERA_TOPICS)


def test_direct_camera_interest_is_bounded():
    interest = {"gimbal": 12.0, "ugv": 9.0, "unknown": 20.0}

    assert requested_camera_keys(0, interest, 10.0) == {"gimbal"}


def test_no_viewer_keeps_display_cameras_idle():
    assert requested_camera_keys(0, {}, 10.0) == set()


def test_stale_or_future_camera_cache_is_not_live_video():
    assert camera_frame_is_fresh(9.0, 10.0, 3.0)
    assert not camera_frame_is_fresh(6.0, 10.0, 3.0)
    assert not camera_frame_is_fresh(None, 10.0, 3.0)
    assert not camera_frame_is_fresh(11.0, 10.0, 3.0)


def test_camera_qos_is_configurable_but_never_queues_old_frames():
    reliable = camera_qos_profile("reliable")
    best_effort = camera_qos_profile("best_effort")

    assert reliable.history == HistoryPolicy.KEEP_LAST
    assert reliable.depth == 1
    assert reliable.reliability == ReliabilityPolicy.RELIABLE
    assert best_effort.reliability == ReliabilityPolicy.BEST_EFFORT
