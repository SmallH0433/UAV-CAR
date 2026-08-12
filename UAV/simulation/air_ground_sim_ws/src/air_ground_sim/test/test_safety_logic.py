from air_ground_sim.safety_logic import (
    Fault,
    Severity,
    evaluate_system_health,
    update_critical_fault_timers,
)


REQUIRED = (
    "mission",
    "mavlink",
    "perception",
    "command_mux",
    "ugv_control_mux",
    "chassis_adapter",
    "ugv_gateway",
)


def healthy_inputs():
    statuses = {
        "mission": {"active": False, "state": "IDLE", "dock_detached": True},
        "mavlink": {
            "connected": True,
            "flight_ready": True,
            "armed": False,
            "landed": True,
            "relative_alt_m": 0.0,
            "battery_remaining_pct": 80,
        },
        "perception": {"healthy": True},
        "command_mux": {"enabled": True},
        "ugv_control_mux": {"enabled": True},
        "chassis_adapter": {"enabled": True, "emergency_stop": False},
        "ugv_gateway": {"enabled": True, "emergency_stop": False},
    }
    ages = {source: 0.1 for source in REQUIRED}
    return statuses, ages


def evaluate(statuses, ages, **overrides):
    arguments = {
        "statuses": statuses,
        "ages_s": ages,
        "source_timeout_s": 1.0,
        "external_estop": False,
        "operator_estop": False,
        "ugv_speed_mps": 0.0,
        "required_sources": REQUIRED,
    }
    arguments.update(overrides)
    return evaluate_system_health(**arguments)


def test_healthy_system_is_ready():
    statuses, ages = healthy_inputs()
    result = evaluate(statuses, ages)
    assert result.ready
    assert result.state == "READY"
    assert result.faults == ()


def test_missing_source_prevents_start_without_startup_latch():
    statuses, ages = healthy_inputs()
    ages.pop("perception")
    result = evaluate(statuses, ages)
    assert not result.ready
    assert not result.has_critical
    assert any(fault.code == "PERCEPTION_STALE" and fault.severity == Severity.WARN for fault in result.faults)


def test_status_loss_while_airborne_is_critical():
    statuses, ages = healthy_inputs()
    statuses["mavlink"]["armed"] = True
    statuses["mavlink"]["relative_alt_m"] = 2.5
    ages["perception"] = 2.0
    result = evaluate(statuses, ages)
    assert result.airborne
    assert result.has_critical
    assert any(fault.code == "PERCEPTION_STALE" for fault in result.faults)


def test_unhealthy_perception_while_airborne_latches_stop_condition():
    statuses, ages = healthy_inputs()
    statuses["mavlink"].update({"armed": True, "relative_alt_m": 3.0})
    statuses["perception"]["healthy"] = False
    result = evaluate(statuses, ages)
    assert any(fault.code == "UAV_PERCEPTION_UNHEALTHY" and fault.severity == Severity.CRITICAL for fault in result.faults)


def test_armed_on_pad_is_not_misclassified_as_airborne():
    statuses, ages = healthy_inputs()
    statuses["mavlink"].update(
        {"armed": True, "landed": True, "relative_alt_m": 0.0}
    )
    statuses["perception"]["healthy"] = False
    result = evaluate(statuses, ages)
    assert not result.airborne
    assert not any(
        fault.code == "UAV_PERCEPTION_UNHEALTHY"
        and fault.severity == Severity.CRITICAL
        for fault in result.faults
    )
    assert not result.ready


def test_armed_aircraft_cannot_be_physically_latched():
    statuses, ages = healthy_inputs()
    statuses["mission"]["dock_detached"] = False
    statuses["mavlink"]["armed"] = True
    result = evaluate(statuses, ages)
    assert any(fault.code == "UAV_ARMED_WHILE_LATCHED" for fault in result.faults)


def test_flight_controller_parameter_drift_blocks_readiness_and_escalates_in_air():
    statuses, ages = healthy_inputs()
    statuses["mavlink"]["required_parameters_verified"] = False
    statuses["mavlink"]["flight_ready"] = False
    result = evaluate(statuses, ages)
    fault = next(
        fault
        for fault in result.faults
        if fault.code == "UAV_PARAMETER_ATTESTATION_FAILED"
    )
    assert fault.severity == Severity.ERROR
    assert not result.ready

    statuses["mavlink"].update(
        {"armed": True, "landed": False, "relative_alt_m": 2.0}
    )
    result = evaluate(statuses, ages)
    fault = next(
        fault
        for fault in result.faults
        if fault.code == "UAV_PARAMETER_ATTESTATION_FAILED"
    )
    assert fault.severity == Severity.CRITICAL


def test_guarded_moving_capture_allows_time_bounded_normal_disarm():
    statuses, ages = healthy_inputs()
    statuses["mission"].update(
        {
            "active": True,
            "state": "LATCH_MOVING",
            "state_elapsed_s": 30.0,
            "dock_detached": False,
            "dock_attached_age_s": 3.0,
        }
    )
    statuses["mavlink"].update(
        {
            "armed": True,
            "landed": False,
            "mode": "LAND",
            "relative_alt_m": 0.35,
        }
    )
    result = evaluate(
        statuses,
        ages,
        moving_capture_armed_timeout_s=8.0,
        moving_capture_max_altitude_m=0.5,
    )
    capture = next(
        fault
        for fault in result.faults
        if fault.code == "UAV_CONTROLLED_CAPTURE_DISARMING"
    )
    assert capture.severity == Severity.WARN
    assert not result.has_critical


def test_guarded_moving_capture_times_out_fail_closed():
    statuses, ages = healthy_inputs()
    statuses["mission"].update(
        {
            "active": True,
            "state": "LATCH_MOVING",
            "state_elapsed_s": 30.0,
            "dock_detached": False,
            "dock_attached_age_s": 8.1,
        }
    )
    statuses["mavlink"].update(
        {
            "armed": True,
            "landed": False,
            "mode": "LAND",
            "relative_alt_m": 0.35,
        }
    )
    result = evaluate(
        statuses,
        ages,
        moving_capture_armed_timeout_s=8.0,
        moving_capture_max_altitude_m=0.5,
    )
    assert any(
        fault.code == "UAV_CAPTURE_DISARM_TIMEOUT"
        and fault.severity == Severity.CRITICAL
        for fault in result.faults
    )


def test_guarded_moving_capture_without_attachment_age_fails_closed():
    statuses, ages = healthy_inputs()
    statuses["mission"].update(
        {
            "active": True,
            "state": "LATCH_MOVING",
            "state_elapsed_s": 1.0,
            "dock_detached": False,
        }
    )
    statuses["mavlink"].update(
        {
            "armed": True,
            "landed": False,
            "mode": "LAND",
            "relative_alt_m": 0.35,
        }
    )
    result = evaluate(statuses, ages)
    assert any(
        fault.code == "UAV_ARMED_WHILE_LATCHED"
        and fault.severity == Severity.CRITICAL
        for fault in result.faults
    )


def test_critical_battery_is_critical_in_flight():
    statuses, ages = healthy_inputs()
    statuses["mavlink"].update(
        {"armed": True, "relative_alt_m": 2.0, "battery_remaining_pct": 9}
    )
    result = evaluate(statuses, ages, critical_battery_pct=10.0)
    assert any(fault.code == "UAV_BATTERY_CRITICAL" and fault.severity == Severity.CRITICAL for fault in result.faults)


def test_external_estop_is_always_critical():
    statuses, ages = healthy_inputs()
    result = evaluate(statuses, ages, external_estop=True)
    assert result.has_critical
    assert result.state == "EMERGENCY_STOP"


def test_mission_fault_is_promoted_to_independent_critical_fault():
    statuses, ages = healthy_inputs()
    statuses["mission"].update(
        {"active": False, "state": "FAULT", "reason": "ugv_navigation_progress_stalled"}
    )
    result = evaluate(statuses, ages)
    fault = next(fault for fault in result.faults if fault.code == "MISSION_FAULT")
    assert fault.severity == Severity.CRITICAL
    assert "ugv_navigation_progress_stalled" in fault.summary
    assert not result.ready


def test_disabled_ground_control_chain_is_not_ready():
    statuses, ages = healthy_inputs()
    statuses["ugv_control_mux"]["enabled"] = False
    result = evaluate(statuses, ages)
    assert not result.ready


def test_required_docking_gateway_is_part_of_real_readiness():
    statuses, ages = healthy_inputs()
    statuses["docking_gateway"] = {
        "enabled": True,
        "healthy": True,
        "critical_fault": "",
    }
    ages["docking_gateway"] = 0.1
    result = evaluate(
        statuses,
        ages,
        required_sources=REQUIRED + ("docking_gateway",),
    )
    assert result.ready


def test_docking_gateway_fault_is_critical_and_blocks_motion():
    statuses, ages = healthy_inputs()
    statuses["docking_gateway"] = {
        "enabled": True,
        "healthy": False,
        "critical_fault": "DOCK_REDUNDANT_CHANNEL_DISAGREEMENT",
    }
    ages["docking_gateway"] = 0.1
    result = evaluate(
        statuses,
        ages,
        required_sources=REQUIRED + ("docking_gateway",),
    )
    assert result.has_critical
    assert any(
        fault.code == "DOCK_REDUNDANT_CHANNEL_DISAGREEMENT"
        for fault in result.faults
    )


def test_downstream_estop_echo_blocks_readiness_without_critical_feedback_loop():
    statuses, ages = healthy_inputs()
    statuses["chassis_adapter"]["emergency_stop"] = True
    result = evaluate(statuses, ages)
    fault = next(fault for fault in result.faults if fault.code == "UGV_EMERGENCY_PATH_ACTIVE")
    assert fault.severity == Severity.ERROR
    assert not result.has_critical


def test_critical_fault_requires_continuous_confirmation_but_estop_is_immediate():
    timers = {}
    perception = Fault("UAV_PERCEPTION_UNHEALTHY", Severity.CRITICAL, "perception", "lost")
    assert update_critical_fault_timers([perception], timers, now_s=10.0, hold_s=1.0) == ()
    assert update_critical_fault_timers([perception], timers, now_s=10.9, hold_s=1.0) == ()
    assert update_critical_fault_timers([perception], timers, now_s=11.0, hold_s=1.0) == ("UAV_PERCEPTION_UNHEALTHY",)
    assert update_critical_fault_timers([], timers, now_s=11.1, hold_s=1.0) == ()
    assert timers == {}
    estop = Fault("EXTERNAL_ESTOP", Severity.CRITICAL, "safety", "pressed")
    assert update_critical_fault_timers(
        [estop], timers, now_s=20.0, hold_s=10.0, immediate_codes=frozenset({"EXTERNAL_ESTOP"})
    ) == ("EXTERNAL_ESTOP",)
