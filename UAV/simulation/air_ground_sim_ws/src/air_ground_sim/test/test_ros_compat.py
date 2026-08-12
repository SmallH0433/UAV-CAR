from air_ground_sim.ros_compat import RCLError, run_shutdown_action


def test_shutdown_action_reports_success():
    called = []
    assert run_shutdown_action(lambda: called.append(True))
    assert called == [True]


def test_shutdown_action_absorbs_only_ros_context_error():
    def closed_context():
        raise RCLError("publisher context is invalid")

    assert not run_shutdown_action(closed_context)


def test_shutdown_action_does_not_hide_programming_errors():
    def broken_action():
        raise ValueError("unexpected shutdown bug")

    try:
        run_shutdown_action(broken_action)
    except ValueError as error:
        assert str(error) == "unexpected shutdown bug"
    else:
        raise AssertionError("non-ROS exceptions must remain visible")
