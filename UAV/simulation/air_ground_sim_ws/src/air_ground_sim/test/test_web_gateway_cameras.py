from air_ground_sim.web_gateway import CAMERA_TOPICS, requested_camera_keys


def test_connected_dashboard_requests_all_camera_streams():
    assert requested_camera_keys(1, {}, 10.0) == set(CAMERA_TOPICS)


def test_direct_camera_interest_is_bounded():
    interest = {"gimbal": 12.0, "ugv": 9.0, "unknown": 20.0}

    assert requested_camera_keys(0, interest, 10.0) == {"gimbal"}


def test_no_viewer_keeps_display_cameras_idle():
    assert requested_camera_keys(0, {}, 10.0) == set()
