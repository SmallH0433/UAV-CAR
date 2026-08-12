import pytest

from air_ground_sim.mission_logic import (
    acknowledged_retry_deadline,
    distance_speed_scale,
    dock_attach_authorized,
    failed_ack_retry_deadline,
    mavlink_command_ack_outcome,
    mission_state_allows_ugv_motion,
    mission_start_is_safe,
    mission_terminal_reset_is_safe,
    mission_plan_is_commissioned,
    MissionFacts,
    MissionState,
    moving_deck_envelope,
    navigation_goal_failed,
    next_state,
    parse_detachable_joint_state,
    progress_watchdog_step,
    split_speed_scale,
    sustained_for,
    transform_stamp_is_fresh,
    update_sustained_since,
)


def test_ground_motion_authority_is_closed_for_every_non_driving_state():
    allowed = {
        MissionState.PARALLEL_TRANSIT,
        MissionState.FOLLOW_MOVING_UGV,
        MissionState.DOCK_MOVING,
        MissionState.LATCH_MOVING,
        MissionState.RIDE_AND_DECELERATE,
    }
    for state in MissionState:
        assert mission_state_allows_ugv_motion(state) is (state in allowed)


def test_mavlink_arm_ack_outcome_distinguishes_retryable_terminal_failure():
    assert mavlink_command_ack_outcome({"command": 400, "result": 0}, 400) == "accepted"
    assert mavlink_command_ack_outcome({"command": 400, "result": 5}, 400) == "in_progress"
    assert mavlink_command_ack_outcome({"command": 400, "result": 4}, 400) == "failed"
    assert mavlink_command_ack_outcome({"command": 22, "result": 4}, 400) is None
    assert mavlink_command_ack_outcome({}, 400) is None


def test_failed_ack_cooldown_uses_monotonic_deadline_without_replaying_old_delay():
    assert failed_ack_retry_deadline(
        now_s=50.0,
        wall_now_s=105.0,
        ack_wall_s=100.0,
        cooldown_s=15.0,
    ) == 60.0
    assert failed_ack_retry_deadline(
        now_s=200.0,
        wall_now_s=130.0,
        ack_wall_s=100.0,
        cooldown_s=15.0,
    ) == 200.0


def test_accepted_ack_confirmation_window_uses_the_same_dual_clock_mapping():
    assert acknowledged_retry_deadline(
        now_s=20.0,
        wall_now_s=1002.0,
        ack_wall_s=1000.0,
        delay_s=10.0,
    ) == 28.0
    assert acknowledged_retry_deadline(
        now_s=50.0,
        wall_now_s=1015.0,
        ack_wall_s=1000.0,
        delay_s=10.0,
    ) == 50.0


def test_real_mission_requires_explicit_commissioning_identity():
    assert mission_plan_is_commissioned(
        simulation_lifecycle=True,
        validated=False,
        plan_id="UNCOMMISSIONED",
    )
    assert not mission_plan_is_commissioned(
        simulation_lifecycle=False,
        validated=False,
        plan_id="SITE-A-REV-3",
    )
    assert not mission_plan_is_commissioned(
        simulation_lifecycle=False,
        validated=True,
        plan_id="UNCOMMISSIONED",
    )
    assert mission_plan_is_commissioned(
        simulation_lifecycle=False,
        validated=True,
        plan_id="SITE-A-REV-3",
    )


def test_stationary_capture_requires_positive_landed_and_disarmed_state():
    assert not dock_attach_authorized(
        MissionState.LATCH_AT_START,
        armed=True,
        landed=True,
        autopilot_mode="LAND",
        altitude_m=0.1,
    )
    assert not dock_attach_authorized(
        MissionState.LATCH_STOPPED,
        armed=False,
        landed=False,
        autopilot_mode="LAND",
        altitude_m=0.1,
    )
    assert dock_attach_authorized(
        MissionState.LATCH_STOPPED,
        armed=False,
        landed=True,
        autopilot_mode="LAND",
        altitude_m=0.1,
    )


def test_moving_capture_requires_final_land_mode_and_low_altitude():
    assert not dock_attach_authorized(
        MissionState.LATCH_MOVING,
        armed=True,
        landed=False,
        autopilot_mode="GUIDED",
        altitude_m=0.3,
    )
    assert not dock_attach_authorized(
        MissionState.LATCH_MOVING,
        armed=True,
        landed=False,
        autopilot_mode="LAND",
        altitude_m=0.8,
        moving_capture_max_altitude_m=0.65,
    )
    assert dock_attach_authorized(
        MissionState.LATCH_MOVING,
        armed=True,
        landed=False,
        autopilot_mode="LAND",
        altitude_m=0.3,
        moving_capture_max_altitude_m=0.65,
    )


def test_initial_docking_sequence_guards():
    state = next_state(
        MissionState.RELEASE_REMOTE_DOCK,
        0.5,
        MissionFacts(dock_detached=True),
    )
    assert state == MissionState.WAIT_AUTOPILOT
    assert next_state(
        state, 0.1, MissionFacts(connected=True, flight_ready=True)
    ) == MissionState.ARM_INITIAL
    assert next_state(MissionState.ARM_INITIAL, 1, MissionFacts(armed=True)) == MissionState.TAKEOFF_INITIAL
    assert next_state(MissionState.TAKEOFF_INITIAL, 2, MissionFacts(altitude_m=2.3)) == MissionState.NAVIGATE_TO_START_DOCK
    assert next_state(
        MissionState.DOCK_AT_START, 3, MissionFacts(docking_capture_ready=True)
    ) == MissionState.LATCH_AT_START


def test_parallel_transit_requires_both_air_and_ground_completion():
    state = MissionState.PARALLEL_TRANSIT
    assert next_state(state, 20, MissionFacts(ugv_goal_done=True)) == state
    assert next_state(state, 20, MissionFacts(navigation_reached=True)) == state
    assert next_state(
        state, 20, MissionFacts(ugv_goal_done=True, navigation_reached=True)
    ) == MissionState.DOCK_STOPPED


def test_moving_landing_has_follow_dwell_and_latch():
    state = MissionState.FOLLOW_MOVING_UGV
    facts = MissionFacts(
        docking_separation_m=2.0,
        ugv_moving=True,
        ugv_motion_envelope=True,
    )
    assert next_state(state, 7.9, facts) == state
    assert next_state(state, 8.0, facts) == MissionState.DOCK_MOVING
    assert (
        next_state(
            state,
            20.0,
            MissionFacts(docking_separation_m=2.0, ugv_moving=False),
        )
        == state
    )
    assert (
        next_state(
            MissionState.DOCK_MOVING,
            5,
            MissionFacts(docking_capture_ready=True, ugv_moving=False),
        )
        == MissionState.DOCK_MOVING
    )
    assert (
        next_state(
            MissionState.DOCK_MOVING,
            5,
            MissionFacts(
                docking_capture_ready=True,
                ugv_moving=True,
                ugv_motion_envelope=True,
            ),
        )
        == MissionState.LATCH_MOVING
    )
    assert next_state(
        MissionState.LATCH_MOVING,
        1,
        MissionFacts(dock_detached=False, landed=True),
    ) == MissionState.RIDE_AND_DECELERATE
    assert next_state(
        MissionState.RIDE_AND_DECELERATE, 10, MissionFacts(ugv_goal_done=True)
    ) == MissionState.RIDE_AND_DECELERATE
    assert next_state(
        MissionState.RIDE_AND_DECELERATE,
        12,
        MissionFacts(ugv_goal_done=True, ugv_stopped_stable=True),
    ) == MissionState.COMPLETE


def test_latch_waits_for_flight_controller_landed_and_disarmed_state():
    still_flying = MissionFacts(armed=True, dock_detached=False, landed=False)
    assert next_state(MissionState.LATCH_AT_START, 4, still_flying) == MissionState.LATCH_AT_START
    assert next_state(MissionState.LATCH_STOPPED, 4, still_flying) == MissionState.LATCH_STOPPED
    assert next_state(MissionState.LATCH_MOVING, 4, still_flying) == MissionState.LATCH_MOVING

    landed = MissionFacts(armed=False, dock_detached=False, landed=True)
    assert next_state(MissionState.LATCH_AT_START, 4, landed) == MissionState.DWELL_AT_START


def test_release_waits_for_joint_ground_state_and_scaled_settle_window():
    ready = MissionFacts(dock_detached=True, landed=True)
    assert (
        next_state(MissionState.RELEASE_FOR_TRANSIT, 1.9, ready)
        == MissionState.RELEASE_FOR_TRANSIT
    )
    assert (
        next_state(MissionState.RELEASE_FOR_TRANSIT, 2.0, ready)
        == MissionState.ARM_FOR_TRANSIT
    )
    assert (
        next_state(
            MissionState.RELEASE_FOR_FOLLOW,
            15.9,
            ready,
            timeout_scale=8.0,
        )
        == MissionState.RELEASE_FOR_FOLLOW
    )
    assert (
        next_state(
            MissionState.RELEASE_FOR_FOLLOW,
            16.0,
            ready,
            timeout_scale=8.0,
        )
        == MissionState.ARM_FOR_FOLLOW
    )
    assert next_state(
        MissionState.RELEASE_FOR_FOLLOW,
        3.0,
        MissionFacts(dock_detached=True, landed=False),
    ) == MissionState.RELEASE_FOR_FOLLOW


def test_state_timeout_faults_closed():
    assert next_state(
        MissionState.DOCK_MOVING, 101.0, MissionFacts()
    ) == MissionState.FAULT


def test_timeout_scale_supports_slow_software_simulation():
    assert next_state(
        MissionState.DOCK_MOVING,
        101.0,
        MissionFacts(),
        timeout_scale=4.0,
    ) == MissionState.DOCK_MOVING


def test_route_profile_can_override_long_running_state_timeout():
    facts = MissionFacts()
    assert next_state(
        MissionState.FOLLOW_MOVING_UGV,
        200.0,
        facts,
        timeout_overrides_s={MissionState.FOLLOW_MOVING_UGV: 240.0},
    ) == MissionState.FOLLOW_MOVING_UGV
    assert next_state(
        MissionState.FOLLOW_MOVING_UGV,
        241.0,
        facts,
        timeout_overrides_s={MissionState.FOLLOW_MOVING_UGV: 240.0},
    ) == MissionState.FAULT


def test_wait_autopilot_requires_position_solution_not_only_heartbeat():
    assert next_state(
        MissionState.WAIT_AUTOPILOT,
        5.0,
        MissionFacts(connected=True, flight_ready=False),
    ) == MissionState.WAIT_AUTOPILOT


def test_gazebo_detachable_joint_string_state_contract():
    assert parse_detachable_joint_state("detached") is True
    assert parse_detachable_joint_state("attached") is False
    assert parse_detachable_joint_state("unknown") is None


def test_latched_ride_speed_scale_depends_on_remaining_distance():
    assert distance_speed_scale(0.15, 0.08, 3.0, 2.0) == 0.15
    assert distance_speed_scale(0.15, 0.08, 1.0, 2.0) == pytest.approx(0.115)
    assert distance_speed_scale(0.15, 0.08, 0.0, 2.0) == 0.08
    assert distance_speed_scale(1.5, -1.0, 1.0, 2.0) == 0.5


def test_moving_deck_requires_heading_and_low_turn_rate():
    safe = {
        "yaw_rad": 3.13,
        "yaw_rate_rps": 0.05,
        "target_yaw_rad": -3.14,
        "max_yaw_error_rad": 0.25,
        "max_yaw_rate_rps": 0.12,
    }
    assert moving_deck_envelope(**safe)
    assert not moving_deck_envelope(**{**safe, "yaw_rad": 2.5})
    assert not moving_deck_envelope(**{**safe, "yaw_rate_rps": 0.2})
    assert not moving_deck_envelope(**{**safe, "yaw_rad": None})


def test_moving_dock_entry_can_be_stricter_than_fail_safe_exit():
    common = {
        "yaw_rad": 2.88,
        "yaw_rate_rps": 0.10,
        "target_yaw_rad": 3.14159,
    }
    assert moving_deck_envelope(
        **common,
        max_yaw_error_rad=0.35,
        max_yaw_rate_rps=0.20,
    )
    assert not moving_deck_envelope(
        **common,
        max_yaw_error_rad=0.20,
        max_yaw_rate_rps=0.12,
    )


def test_terminal_nav2_failure_is_not_left_to_a_long_state_timeout():
    assert navigation_goal_failed("ended_6")
    assert navigation_goal_failed("rejected")
    assert navigation_goal_failed("send_error:transport")
    assert not navigation_goal_failed("executing")


def test_map_to_base_transform_must_be_recent_and_from_current_clock_epoch():
    now = 10_000_000_000
    assert transform_stamp_is_fresh(
        now_ns=now, stamp_ns=9_800_000_000, timeout_s=0.25
    )
    assert not transform_stamp_is_fresh(
        now_ns=now, stamp_ns=9_700_000_000, timeout_s=0.25
    )
    assert not transform_stamp_is_fresh(
        now_ns=now, stamp_ns=10_200_000_000, timeout_s=0.25
    )
    assert not transform_stamp_is_fresh(now_ns=now, stamp_ns=0, timeout_s=0.25)


def test_speed_scale_is_upstream_limit_plus_binary_safety_gate():
    assert split_speed_scale(0.15) == (0.15, 15.0, 1.0)
    assert split_speed_scale(2.0) == (1.0, 100.0, 1.0)
    # In nav2_msgs/SpeedLimit, zero means "no limit". The downstream gate is
    # therefore what makes zero a fail-safe stop.
    assert split_speed_scale(0.0) == (0.0, 0.0, 0.0)
    assert split_speed_scale(-1.0) == (0.0, 0.0, 0.0)


def test_hazardous_transition_requires_continuous_readiness():
    since = update_sustained_since(True, 0.0, 10.0)
    assert since == 10.0
    assert not sustained_for(True, since, 12.9, 3.0)
    assert sustained_for(True, since, 13.0, 3.0)
    assert update_sustained_since(True, since, 14.0) == since
    assert update_sustained_since(False, since, 14.1) == 0.0
    assert not sustained_for(False, since, 20.0, 3.0)


def test_progress_watchdog_allows_slow_progress_but_detects_stall_and_pose_loss():
    anchor, since, stalled = progress_watchdog_step(
        position_xy=(1.0, 2.0),
        anchor_xy=None,
        anchor_since_s=None,
        now_s=10.0,
        minimum_progress_m=0.2,
        timeout_s=5.0,
    )
    assert anchor == (1.0, 2.0)
    assert since == 10.0
    assert not stalled

    anchor, since, stalled = progress_watchdog_step(
        position_xy=(1.21, 2.0),
        anchor_xy=anchor,
        anchor_since_s=since,
        now_s=14.9,
        minimum_progress_m=0.2,
        timeout_s=5.0,
    )
    assert anchor == (1.21, 2.0)
    assert since == 14.9
    assert not stalled

    _, _, stalled = progress_watchdog_step(
        position_xy=None,
        anchor_xy=anchor,
        anchor_since_s=since,
        now_s=20.0,
        minimum_progress_m=0.2,
        timeout_s=5.0,
    )
    assert stalled


def test_start_and_terminal_reset_require_positive_quiescent_state():
    assert mission_start_is_safe(
        MissionState.IDLE,
        armed=False,
        landed=True,
        ugv_speed_mps=0.01,
        stopped_speed_mps=0.03,
    )
    assert not mission_start_is_safe(
        MissionState.FAULT,
        armed=False,
        landed=True,
        ugv_speed_mps=0.0,
        stopped_speed_mps=0.03,
    )
    assert not mission_terminal_reset_is_safe(
        MissionState.FAULT,
        armed=True,
        landed=True,
        ugv_speed_mps=0.0,
        stopped_speed_mps=0.03,
    )
    assert not mission_terminal_reset_is_safe(
        MissionState.FAULT,
        armed=False,
        landed=True,
        ugv_speed_mps=0.04,
        stopped_speed_mps=0.03,
    )
    assert mission_terminal_reset_is_safe(
        MissionState.FAULT,
        armed=False,
        landed=True,
        ugv_speed_mps=0.0,
        stopped_speed_mps=0.03,
    )
