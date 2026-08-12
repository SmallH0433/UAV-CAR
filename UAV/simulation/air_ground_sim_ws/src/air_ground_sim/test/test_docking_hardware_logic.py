from air_ground_sim.docking_hardware_logic import (
    dual_channel_state,
    feedback_safe_for_enable,
    operation_command_decision,
    physical_attach_authorized,
    physical_release_authorized,
)


def attach(**overrides):
    facts = {
        "mission_state": "LATCH_STOPPED",
        "contact_a": True,
        "contact_b": True,
        "armed": False,
        "landed": True,
        "autopilot_mode": "LAND",
        "altitude_m": 0.1,
        "ugv_speed_mps": 0.0,
        "ugv_yaw_rate_rps": 0.0,
        "stationary_speed_limit_mps": 0.03,
        "moving_speed_limit_mps": 0.15,
        "moving_yaw_rate_limit_rps": 0.12,
        "moving_capture_max_altitude_m": 0.5,
    }
    facts.update(overrides)
    return physical_attach_authorized(**facts)


def test_dual_feedback_disagreement_is_unknown():
    assert dual_channel_state(True, True) is True
    assert dual_channel_state(False, False) is False
    assert dual_channel_state(True, False) is None


def test_repeated_command_cannot_extend_actuator_timeout():
    publish, started = operation_command_decision(
        commanded_locked=True,
        requested_locked=True,
        feedback_locked=False,
        operation_started_s=10.0,
        now_s=11.5,
    )
    assert publish
    assert started == 10.0


def test_confirmed_command_is_not_republished():
    publish, started = operation_command_decision(
        commanded_locked=True,
        requested_locked=True,
        feedback_locked=True,
        operation_started_s=10.0,
        now_s=11.5,
    )
    assert not publish
    assert started == 0.0


def test_enable_requires_consistent_redundant_feedback():
    assert feedback_safe_for_enable(contact_state=False, locked_state=False)
    assert feedback_safe_for_enable(contact_state=True, locked_state=True)
    assert not feedback_safe_for_enable(contact_state=None, locked_state=False)
    assert not feedback_safe_for_enable(contact_state=True, locked_state=None)
    assert not feedback_safe_for_enable(contact_state=False, locked_state=True)


def test_stationary_lock_requires_contact_landed_disarmed_and_stopped():
    assert attach()
    assert not attach(contact_b=False)
    assert not attach(armed=True)
    assert not attach(landed=False)
    assert not attach(ugv_speed_mps=0.04)


def test_moving_lock_requires_land_envelope_and_bounded_vehicle_speed():
    moving = {
        "mission_state": "LATCH_MOVING",
        "armed": True,
        "landed": False,
        "autopilot_mode": "LAND",
        "altitude_m": 0.4,
        "ugv_speed_mps": 0.1,
    }
    assert attach(**moving)
    assert not attach(**{**moving, "autopilot_mode": "GUIDED"})
    assert not attach(**{**moving, "altitude_m": 0.6})
    assert not attach(**{**moving, "ugv_speed_mps": 0.2})
    assert not attach(**{**moving, "ugv_yaw_rate_rps": 0.2})


def test_release_requires_explicit_state_and_safe_vehicle_states():
    facts = {
        "mission_state": "RELEASE_FOR_FOLLOW",
        "armed": False,
        "landed": True,
        "ugv_speed_mps": 0.0,
        "stationary_speed_limit_mps": 0.03,
    }
    assert physical_release_authorized(**facts)
    assert not physical_release_authorized(**{**facts, "armed": True})
    assert not physical_release_authorized(**{**facts, "mission_state": "DOCK_MOVING"})
