from types import SimpleNamespace

from pymavlink import mavutil

from air_ground_sim.uav_mavlink_bridge import (
    flight_telemetry_ready,
    is_flight_controller_heartbeat,
    lifecycle_operation_enabled,
    mavlink_landed_state,
    mavlink_enum_name,
    parameter_attestation,
    parse_required_parameters,
    required_telemetry_intervals,
    sys_status_prearm_passed,
    velocity_forwarding_enable_allowed,
)


def test_flight_operation_permissions_are_separated():
    permissions = {
        "allow_lifecycle_commands": False,
        "allow_mode_commands": True,
        "allow_land_command": True,
    }
    assert lifecycle_operation_enabled("guided", **permissions)
    assert lifecycle_operation_enabled("land", **permissions)
    assert not lifecycle_operation_enabled("arm", **permissions)
    assert not lifecycle_operation_enabled("takeoff", **permissions)
    assert not lifecycle_operation_enabled("disarm", **permissions)


def test_flight_operations_fail_closed_without_profile_permissions():
    permissions = {
        "allow_lifecycle_commands": False,
        "allow_mode_commands": False,
        "allow_land_command": False,
    }
    for operation in ("guided", "loiter", "land", "arm", "takeoff", "disarm"):
        assert not lifecycle_operation_enabled(operation, **permissions)


def test_velocity_streaming_is_mutually_exclusive_with_ground_lifecycle_control():
    nominal = {
        "emergency_stop": False,
        "connected": True,
        "armed": True,
        "landed": False,
        "mode": "GUIDED",
    }
    assert velocity_forwarding_enable_allowed(**nominal)
    assert not velocity_forwarding_enable_allowed(**{**nominal, "armed": False})
    assert not velocity_forwarding_enable_allowed(**{**nominal, "landed": True})
    assert not velocity_forwarding_enable_allowed(**{**nominal, "landed": None})
    assert not velocity_forwarding_enable_allowed(**{**nominal, "mode": "LAND"})
    assert not velocity_forwarding_enable_allowed(
        **{**nominal, "emergency_stop": True}
    )


def heartbeat(autopilot, vehicle_type):
    return SimpleNamespace(autopilot=autopilot, type=vehicle_type)


def test_accepts_ardupilot_copter_heartbeat():
    message = heartbeat(
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
    )
    assert is_flight_controller_heartbeat(message)


def test_rejects_mission_planner_gcs_heartbeat():
    message = heartbeat(
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        mavutil.mavlink.MAV_TYPE_GCS,
    )
    assert not is_flight_controller_heartbeat(message)


def test_rejects_companion_computer_heartbeat():
    message = heartbeat(
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
    )
    assert not is_flight_controller_heartbeat(message)


def test_mavlink_enum_name_is_readable_and_tolerates_unknown_values():
    assert "ACCEPTED" in mavlink_enum_name("MAV_RESULT", 0)
    assert mavlink_enum_name("MAV_RESULT", 987654) == "987654"


def test_prearm_status_requires_enabled_and_healthy_bits():
    bit = int(getattr(mavutil.mavlink, "MAV_SYS_STATUS_PREARM_CHECK", 1 << 28))
    assert sys_status_prearm_passed(
        SimpleNamespace(
            onboard_control_sensors_enabled=bit,
            onboard_control_sensors_health=bit,
        )
    )
    assert not sys_status_prearm_passed(
        SimpleNamespace(
            onboard_control_sensors_enabled=bit,
            onboard_control_sensors_health=0,
        )
    )


def test_extended_system_landed_state_is_fail_closed():
    assert mavlink_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND) is True
    assert mavlink_landed_state(mavutil.mavlink.MAV_LANDED_STATE_IN_AIR) is False
    assert mavlink_landed_state(mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED) is None


def test_flight_readiness_requires_acknowledged_complete_telemetry():
    telemetry = {
        "prearm_checks_passed": True,
        "fix_type": 6,
        "local_position_enu_m": [0.0, 0.0, 0.0],
        "relative_alt_m": 0.0,
    }
    assert flight_telemetry_ready(
        telemetry, connected=True, streams_configured=True
    )
    assert not flight_telemetry_ready(
        telemetry, connected=True, streams_configured=False
    )
    assert not flight_telemetry_ready(
        {**telemetry, "prearm_checks_passed": False},
        connected=True,
        streams_configured=True,
    )
    assert not flight_telemetry_ready(
        {**telemetry, "local_position_enu_m": None},
        connected=True,
        streams_configured=True,
    )
    assert not flight_telemetry_ready(
        telemetry,
        connected=True,
        streams_configured=True,
        parameters_verified=False,
    )


def test_required_parameter_policy_is_strict_and_deterministic():
    assert parse_required_parameters(
        '{"DISARM_DELAY": 5, "ARMING_CHECK": 1.0}'
    ) == {"ARMING_CHECK": 1.0, "DISARM_DELAY": 5.0}

    for invalid in (
        "[]",
        '{"lowercase": 1}',
        '{"PARAMETER_NAME_TOO_LONG": 1}',
        '{"ARMING_CHECK": true}',
        '{"ARMING_CHECK": "1"}',
    ):
        try:
            parse_required_parameters(invalid)
        except ValueError:
            pass
        else:
            raise AssertionError(f"Expected invalid parameter policy: {invalid}")


def test_parameter_attestation_fails_closed_until_every_value_matches():
    required = {"ARMING_CHECK": 1.0, "DISARM_DELAY": 5.0}
    verified, report = parameter_attestation(
        required,
        {"ARMING_CHECK": 1.0},
        tolerance=0.001,
    )
    assert not verified
    assert report["ARMING_CHECK"]["matched"]
    assert report["DISARM_DELAY"]["actual"] is None

    verified, report = parameter_attestation(
        required,
        {"ARMING_CHECK": 1.0, "DISARM_DELAY": 5.0005},
        tolerance=0.001,
    )
    assert verified
    assert all(item["matched"] for item in report.values())

    verified, report = parameter_attestation(
        required,
        {"ARMING_CHECK": 0.0, "DISARM_DELAY": 5.0},
        tolerance=0.001,
    )
    assert not verified
    assert report["ARMING_CHECK"]["actual"] == 0.0


def test_required_telemetry_stream_contract_has_unique_message_ids():
    intervals = required_telemetry_intervals()
    assert len(intervals) == 7
    assert len({message_id for message_id, _ in intervals}) == len(intervals)
    assert all(interval_us > 0 for _, interval_us in intervals)
