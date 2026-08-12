from air_ground_sim.runtime_acceptance import (
    EXPECTED_SEQUENCE,
    REQUIRED_OBSERVED_SEQUENCE,
    ordered_subsequence,
    planar_path_length,
    snapshot_violations,
)


def healthy_snapshot(state="PARALLEL_TRANSIT", gate=True):
    sensors = {
        name: {"healthy": True}
        for name in (
            "gimbal_camera",
            "stereo_left",
            "stereo_right",
            "stereo_depth",
            "lidar2d",
            "lidar3d",
            "ultrasonic_front",
            "ultrasonic_rear",
            "ultrasonic_left",
            "ultrasonic_right",
            "ultrasonic_up",
            "ultrasonic_down",
        )
    }
    return {
        "mission": {
            "active": state != "COMPLETE",
            "state": state,
            "ugv_safety_gate_open": gate,
            "dock_detached": False,
            "mission_plan": {
                "commissioned": True,
                "id": "SIL-AIR-GROUND-COOP-V1",
            },
        },
        "system": {
            "state": "READY",
            "ready": True,
            "latched": False,
            "faults": [],
            "emergency_stop": False,
        },
        "mavlink": {
            "connected": True,
            "armed": state != "COMPLETE",
            "landed": state == "COMPLETE",
            "required_parameters_verified": True,
            "telemetry_streams_configured": True,
            "telemetry_stream_acknowledged": 7,
            "telemetry_stream_required": 7,
        },
        "perception": {"healthy": True, "sensors": sensors},
        "docking": {"active": False},
        "ugv": {"speed_mps": 0.0},
        "ugv_control_mux": {"gate_open": gate},
        "cameras": {
            name: {"ready": True}
            for name in (
                "gimbal",
                "stereo_left",
                "stereo_right",
                "landing",
                "downward",
                "ugv",
            )
        },
        "paths": {"ugv_global": [[0.0, 0.0], [-1.0, 0.0], [-2.0, 0.0]]},
    }


def test_healthy_driving_snapshot_passes():
    assert snapshot_violations(healthy_snapshot()) == []


def test_non_driving_state_rejects_inherited_ground_gate():
    assert "UGV_GATE_OPEN_IN_NON_DRIVING_STATE" in snapshot_violations(
        healthy_snapshot("DOCK_STOPPED", gate=True)
    )


def test_safe_prearm_warning_is_tolerated_but_motion_and_errors_fail():
    waiting = healthy_snapshot("ARM_INITIAL", gate=False)
    waiting["mavlink"]["armed"] = False
    waiting["system"].update(
        {
            "state": "DEGRADED",
            "ready": False,
            "faults": [{"code": "UAV_PREFLIGHT_NOT_READY", "level": 1}],
        }
    )
    assert snapshot_violations(waiting) == []

    moving = healthy_snapshot("PARALLEL_TRANSIT", gate=True)
    moving["system"]["ready"] = False
    assert "SYSTEM_NOT_READY_DURING_MOTION" in snapshot_violations(moving)

    waiting["system"]["faults"] = [{"code": "CRITICAL_IO", "level": 2}]
    assert "SYSTEM_CRITICAL_FAULT_PRESENT" in snapshot_violations(waiting)


def test_final_acceptance_requires_safe_terminal_state():
    assert snapshot_violations(healthy_snapshot("COMPLETE", gate=False)) == []
    unsafe = healthy_snapshot("COMPLETE", gate=True)
    unsafe["mavlink"]["armed"] = True
    violations = snapshot_violations(unsafe)
    assert "FINAL_UAV_ARMED" in violations
    assert "FINAL_UGV_GATE_OPEN" in violations


def test_state_sequence_check_rejects_missing_milestone():
    assert ordered_subsequence(EXPECTED_SEQUENCE, EXPECTED_SEQUENCE)
    assert not ordered_subsequence(
        [state for state in EXPECTED_SEQUENCE if state != "LATCH_MOVING"],
        EXPECTED_SEQUENCE,
    )
    assert "WAIT_AUTOPILOT" not in REQUIRED_OBSERVED_SEQUENCE
    assert "RELEASE_REMOTE_DOCK" not in REQUIRED_OBSERVED_SEQUENCE
    assert ordered_subsequence(
        ["IDLE", *REQUIRED_OBSERVED_SEQUENCE], REQUIRED_OBSERVED_SEQUENCE
    )


def test_ride_acceptance_rejects_dubins_loop_hidden_by_euclidean_distance():
    assert planar_path_length([[0, 0], [3, 4]]) == 5.0
    snapshot = healthy_snapshot("RIDE_AND_DECELERATE", gate=True)
    snapshot["mission"]["ride_remaining_distance_m"] = 1.0
    snapshot["paths"]["ugv_global"] = [[0.0, 0.0], [0.0, 5.0], [1.0, 5.0]]
    assert "RIDE_PATH_NONHOLONOMIC_DETOUR" in snapshot_violations(snapshot)
