import math
import sys
import unittest
from pathlib import Path

from air_ground_landing.guided_execution import (
    LandingSwitchConfig,
    LandingSwitchState,
    HorizontalVelocityLimitConfig,
    HorizontalVelocityLimiter,
    ModeTransitionConfig,
    ModeTransitionManager,
    ModeTransitionPhase,
    RcAuthorizationGate,
    RcGateConfig,
    RcGateState,
    RcLandingRequestGate,
)
from air_ground_landing.follow_tone_policy import (
    EXIT_CONFIRMED_TUNE,
    FOLLOW_ACTIVE_TUNE,
    LANDING_ACTIVE_TUNE,
    OBSERVE_READY_TUNE,
    FollowToneEvent,
    FollowTonePolicy,
)
from air_ground_landing.hybrid_guidance import ControlOwner
from air_ground_landing.mavros_frames import body_frd_pose_to_ros_baselink
from air_ground_landing.simple_coordination import (
    SimpleCoordinationConfig,
    select_simple_owner,
)


WORKSPACE = Path(__file__).resolve().parents[3]
OV9281_DIR = WORKSPACE / "ov9281_debug"
sys.path.insert(0, str(OV9281_DIR))
sys.path.insert(0, str(WORKSPACE / "tools"))

from ov9281_dual_tag import (  # noqa: E402
    parse_tag_quality_specs,
    parse_tag_specs,
    select_primary_tag,
)
from generate_ov9281_nested_apriltag_pdf import INNER_ID1, OUTER_ID0  # noqa: E402


class GuidedExecutionTests(unittest.TestCase):
    def test_follow_tones_require_echo_and_confirmed_exit(self):
        policy = FollowTonePolicy()
        self.assertEqual(
            policy.update(
                observe_ready=True,
                follow_active=False,
                landing_active=False,
                exit_confirmed=False,
                now_s=0.0,
            ),
            (FollowToneEvent.OBSERVE_READY,),
        )
        self.assertEqual(
            policy.update(
                observe_ready=True,
                follow_active=False,
                landing_active=False,
                exit_confirmed=False,
                now_s=0.1,
            ),
            (),
        )
        self.assertEqual(
            policy.update(
                observe_ready=True,
                follow_active=True,
                landing_active=False,
                exit_confirmed=False,
                now_s=0.2,
            ),
            (FollowToneEvent.FOLLOW_ACTIVE,),
        )
        self.assertEqual(
            policy.update(
                observe_ready=True,
                follow_active=False,
                landing_active=True,
                exit_confirmed=False,
                now_s=0.3,
            ),
            (FollowToneEvent.LANDING_ACTIVE,),
        )
        self.assertEqual(
            policy.update(
                observe_ready=False,
                follow_active=False,
                landing_active=False,
                exit_confirmed=True,
                now_s=0.4,
            ),
            (FollowToneEvent.EXIT_CONFIRMED,),
        )
        self.assertNotEqual(OBSERVE_READY_TUNE, FOLLOW_ACTIVE_TUNE)
        self.assertEqual(LANDING_ACTIVE_TUNE, EXIT_CONFIRMED_TUNE)

    def test_service_ack_heartbeat_ack_and_rollback_are_distinct(self):
        manager = ModeTransitionManager(
            ModeTransitionConfig(
                target_ack_timeout_s=2.0,
                rollback_ack_timeout_s=2.0,
                fallback_mode="LOITER",
            )
        )
        request = manager.update(now_s=0.0, current_mode="LOITER", desired_mode="GUIDED")
        self.assertEqual(request.mode, "GUIDED")
        self.assertFalse(manager.status().setpoint_stream_authorized)

        manager.on_service_result(sequence=request.sequence, mode_sent=True, now_s=0.1)
        self.assertEqual(manager.status().phase, ModeTransitionPhase.WAITING_TARGET_HEARTBEAT)
        self.assertTrue(manager.status().mavros_service_ack)
        self.assertFalse(manager.status().heartbeat_ack)

        manager.update(now_s=0.2, current_mode="GUIDED", desired_mode="GUIDED")
        self.assertTrue(manager.status().setpoint_stream_authorized)
        rollback = manager.update(now_s=0.3, current_mode="GUIDED", desired_mode=None)
        self.assertTrue(rollback.rollback)
        self.assertEqual(rollback.mode, "LOITER")
        manager.on_service_result(sequence=rollback.sequence, mode_sent=True, now_s=0.4)
        manager.update(now_s=0.5, current_mode="LOITER", desired_mode=None)
        self.assertEqual(manager.status().phase, ModeTransitionPhase.IDLE)
        self.assertFalse(manager.status().setpoint_stream_authorized)

    def test_horizontal_velocity_is_speed_and_acceleration_limited(self):
        limiter = HorizontalVelocityLimiter(
            HorizontalVelocityLimitConfig(
                maximum_speed_mps=0.10,
                maximum_acceleration_mps2=0.15,
            )
        )
        self.assertEqual(limiter.apply(1.0, 0.0, now_s=0.0), (0.0, 0.0))
        vx, vy = limiter.apply(1.0, 0.0, now_s=0.1)
        self.assertAlmostEqual(vx, 0.015)
        self.assertAlmostEqual(vy, 0.0)
        vx, vy = limiter.apply(1.0, 1.0, now_s=1.1)
        self.assertLessEqual(math.hypot(vx, vy), 0.10 + 1.0e-9)

    def test_failed_rollback_retries_after_service_recovers(self):
        manager = ModeTransitionManager(
            ModeTransitionConfig(
                target_ack_timeout_s=2.0,
                rollback_ack_timeout_s=2.0,
                rollback_retry_interval_s=1.0,
                fallback_mode="LOITER",
            )
        )
        guided = manager.update(now_s=0.0, current_mode="LOITER", desired_mode="GUIDED")
        manager.on_service_result(sequence=guided.sequence, mode_sent=True, now_s=0.1)
        manager.update(now_s=0.2, current_mode="GUIDED", desired_mode="GUIDED")
        rollback = manager.update(now_s=0.3, current_mode="GUIDED", desired_mode=None)
        manager.on_service_result(sequence=rollback.sequence, mode_sent=False, now_s=0.4)
        self.assertEqual(manager.status().phase, ModeTransitionPhase.FAULT)
        self.assertIsNone(manager.update(now_s=1.3, current_mode="GUIDED", desired_mode=None))
        retry = manager.update(now_s=1.4, current_mode="GUIDED", desired_mode=None)
        self.assertIsNotNone(retry)
        self.assertTrue(retry.rollback)
        self.assertEqual(retry.mode, "LOITER")

    def test_rc_gate_fails_closed(self):
        gate = RcAuthorizationGate(RcGateConfig(channel=8, maximum_age_s=0.5))
        channels = [1500] * 7 + [1900]
        self.assertEqual(
            gate.evaluate(channels, received_time_s=1.0, now_s=1.1).state,
            RcGateState.AUTHORIZED,
        )
        self.assertEqual(
            gate.evaluate(channels, received_time_s=1.0, now_s=1.6).state,
            RcGateState.STALE,
        )
        channels[-1] = 1100
        self.assertEqual(
            gate.evaluate(channels, received_time_s=2.0, now_s=2.1).state,
            RcGateState.ABORT,
        )

    def test_swd_requires_confirmed_follow_and_a_low_to_high_edge(self):
        gate = RcLandingRequestGate(
            LandingSwitchConfig(channel=8, maximum_age_s=0.5)
        )
        channels = [1500] * 7 + [1900]

        self.assertEqual(
            gate.evaluate(
                channels,
                received_time_s=1.0,
                now_s=1.1,
                follow_active=False,
            ).state,
            LandingSwitchState.FOLLOW_INACTIVE,
        )
        self.assertEqual(
            gate.evaluate(
                channels,
                received_time_s=1.0,
                now_s=1.1,
                follow_active=True,
            ).state,
            LandingSwitchState.NEEDS_REARM,
        )

        channels[-1] = 1100
        ready = gate.evaluate(
            channels,
            received_time_s=1.2,
            now_s=1.2,
            follow_active=True,
        )
        self.assertEqual(ready.state, LandingSwitchState.READY)
        self.assertFalse(ready.requested)

        channels[-1] = 1900
        requested = gate.evaluate(
            channels,
            received_time_s=1.3,
            now_s=1.3,
            follow_active=True,
        )
        self.assertEqual(requested.state, LandingSwitchState.REQUESTED)
        self.assertTrue(requested.requested)
        self.assertTrue(gate.requested)

        channels[-1] = 1100
        cancelled = gate.evaluate(
            channels,
            received_time_s=1.4,
            now_s=1.4,
            follow_active=True,
        )
        self.assertEqual(cancelled.state, LandingSwitchState.READY)
        self.assertFalse(cancelled.requested)

    def test_confirmed_guided_land_guided_switch_preserves_entry_rollback(self):
        manager = ModeTransitionManager(
            ModeTransitionConfig(
                target_ack_timeout_s=2.0,
                rollback_ack_timeout_s=2.0,
                fallback_mode="LOITER",
            )
        )
        guided = manager.update(
            now_s=0.0,
            current_mode="ALT_HOLD",
            desired_mode="GUIDED",
        )
        manager.on_service_result(sequence=guided.sequence, mode_sent=True, now_s=0.1)
        manager.update(now_s=0.2, current_mode="GUIDED", desired_mode="GUIDED")

        land = manager.update(now_s=0.3, current_mode="GUIDED", desired_mode="LAND")
        self.assertEqual(land.mode, "LAND")
        self.assertFalse(land.rollback)
        self.assertEqual(land.reason, "REQUEST_CHANGED_TARGET_MODE")
        self.assertEqual(manager.status().rollback_mode, "ALT_HOLD")


        manager.on_service_result(sequence=land.sequence, mode_sent=True, now_s=0.4)
        manager.update(now_s=0.5, current_mode="LAND", desired_mode="LAND")

        resume = manager.update(now_s=0.6, current_mode="LAND", desired_mode="GUIDED")
        self.assertEqual(resume.mode, "GUIDED")
        self.assertFalse(resume.rollback)
        manager.on_service_result(sequence=resume.sequence, mode_sent=True, now_s=0.7)
        manager.update(now_s=0.8, current_mode="GUIDED", desired_mode="GUIDED")
        self.assertEqual(manager.status().phase, ModeTransitionPhase.ACTIVE)
        self.assertEqual(manager.status().rollback_mode, "ALT_HOLD")

    def test_swd_cancel_during_land_transition_keeps_confirmed_guided(self):
        manager = ModeTransitionManager(
            ModeTransitionConfig(
                target_ack_timeout_s=2.0,
                rollback_ack_timeout_s=2.0,
                fallback_mode="LOITER",
            )
        )
        guided = manager.update(
            now_s=0.0,
            current_mode="ALT_HOLD",
            desired_mode="GUIDED",
        )
        manager.on_service_result(sequence=guided.sequence, mode_sent=True, now_s=0.1)
        manager.update(now_s=0.2, current_mode="GUIDED", desired_mode="GUIDED")
        land = manager.update(now_s=0.3, current_mode="GUIDED", desired_mode="LAND")
        self.assertEqual(land.mode, "LAND")

        request = manager.update(
            now_s=0.4,
            current_mode="GUIDED",
            desired_mode="GUIDED",
        )
        self.assertIsNone(request)
        self.assertEqual(manager.status().phase, ModeTransitionPhase.ACTIVE)
        self.assertEqual(manager.status().target_mode, "GUIDED")
        self.assertTrue(manager.status().setpoint_stream_authorized)
        self.assertEqual(manager.status().rollback_mode, "ALT_HOLD")


class SimpleCoordinationTests(unittest.TestCase):
    def setUp(self):
        self.config = SimpleCoordinationConfig(
            ibvs_timeout_s=0.25,
            landing_target_timeout_s=0.35,
        )

    def decide(self, **overrides):
        values = {
            "connected": True,
            "descent_requested": False,
            "ibvs_age_s": 0.05,
            "landing_target_age_s": 0.05,
            "landing_target_healthy": True,
            "config": self.config,
        }
        values.update(overrides)
        return select_simple_owner(**values)

    def test_follow_uses_ibvs_when_connected_and_fresh(self):
        decision = self.decide()
        self.assertEqual(decision.owner, ControlOwner.IBVS_GUIDED)

    def test_swd_handover_requires_fresh_healthy_landing_target(self):
        decision = self.decide(descent_requested=True)
        self.assertEqual(decision.owner, ControlOwner.AC_PRECLAND_LAND)
        blocked = self.decide(
            descent_requested=True,
            landing_target_age_s=0.5,
        )
        self.assertEqual(blocked.owner, ControlOwner.HOLD)

    def test_disconnected_or_stale_follow_fails_closed(self):
        self.assertEqual(
            self.decide(connected=False).owner,
            ControlOwner.HOLD,
        )
        self.assertEqual(
            self.decide(ibvs_age_s=0.5).owner,
            ControlOwner.HOLD,
        )


class MavrosLandingTargetFrameTests(unittest.TestCase):
    def test_frd_pose_is_preconverted_to_ros_flu_for_mavros(self):
        position, orientation = body_frd_pose_to_ros_baselink(
            (1.0, 0.2, 0.3),
            (1.0, 0.0, 0.0, 0.0),
        )
        self.assertEqual(position, (1.0, -0.2, -0.3))
        self.assertEqual(orientation, (0.0, 1.0, 0.0, 0.0))

    def test_nonfinite_pose_is_rejected(self):
        with self.assertRaises(ValueError):
            body_frd_pose_to_ros_baselink(
                (1.0, float("nan"), 0.3),
                (1.0, 0.0, 0.0, 0.0),
            )


class DualTagPolicyTests(unittest.TestCase):
    def test_nested_print_pattern_decodes_both_ids(self):
        try:
            import cv2
            import numpy as np
        except ImportError as exc:
            self.skipTest(str(exc))

        def raster(matrix, cell_px):
            base = np.array(
                [[0 if value == "1" else 255 for value in row] for row in matrix],
                dtype=np.uint8,
            )
            return np.kron(base, np.ones((cell_px, cell_px), dtype=np.uint8))

        outer = raster(OUTER_ID0, 100)
        canvas = np.full((1000, 1000), 255, dtype=np.uint8)
        canvas[100:900, 100:900] = outer
        inner = raster(INNER_ID1, 20)
        patch = np.full((200, 200), 255, dtype=np.uint8)
        patch[20:180, 20:180] = inner
        diagonal = 288
        source = np.full((diagonal, diagonal), 255, dtype=np.uint8)
        mask = np.zeros((diagonal, diagonal), dtype=np.uint8)
        offset = (diagonal - 200) // 2
        source[offset : offset + 200, offset : offset + 200] = patch
        mask[offset : offset + 200, offset : offset + 200] = 255
        transform = cv2.getRotationMatrix2D((diagonal / 2, diagonal / 2), 45.0, 1.0)
        rotated = cv2.warpAffine(
            source, transform, (diagonal, diagonal), flags=cv2.INTER_NEAREST, borderValue=255
        )
        rotated_mask = cv2.warpAffine(
            mask, transform, (diagonal, diagonal), flags=cv2.INTER_NEAREST, borderValue=0
        )
        start = (1000 - diagonal) // 2
        region = canvas[start : start + diagonal, start : start + diagonal]
        region[rotated_mask > 0] = rotated[rotated_mask > 0]
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        parameters = cv2.aruco.DetectorParameters_create()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        _, identifiers, _ = cv2.aruco.detectMarkers(canvas, dictionary, parameters=parameters)
        self.assertIsNotNone(identifiers)
        self.assertEqual(set(int(value) for value in identifiers.ravel()), {0, 1})

    def test_dual_sizes_and_hysteretic_primary_selection(self):
        specs = parse_tag_specs("0:0.100:outer,1:0.020:inner")
        self.assertEqual(specs[0].size_m, 0.1)
        self.assertEqual(specs[1].size_m, 0.02)
        outer = {
            "tag_id": 0,
            "role": "outer",
            "distance_m": 0.38,
            "decision_margin": 80.0,
        }
        inner = {
            "tag_id": 1,
            "role": "inner",
            "distance_m": 0.38,
            "decision_margin": 60.0,
        }
        self.assertEqual(
            select_primary_tag(
                (outer, inner),
                previous_tag_id=0,
                switch_to_inner_below_m=0.35,
                hysteresis_m=0.05,
            )["tag_id"],
            0,
        )
        self.assertEqual(
            select_primary_tag(
                (outer, inner),
                previous_tag_id=1,
                switch_to_inner_below_m=0.35,
                hysteresis_m=0.05,
            )["tag_id"],
            1,
        )
        close_inner = dict(inner, distance_m=0.25)
        self.assertEqual(
            select_primary_tag(
                (outer, close_inner),
                previous_tag_id=0,
                switch_to_inner_below_m=0.35,
                hysteresis_m=0.05,
            )["tag_id"],
            1,
        )

    def test_outer_first_uses_per_tag_quality_and_falls_back_to_inner(self):
        gates = parse_tag_quality_specs("0:50:2:1.0,1:35:2:1.5")
        outer = {
            "tag_id": 0,
            "role": "outer",
            "distance_m": 0.55,
            "decision_margin": 57.0,
            "hamming": 2,
            "reprojection_error_px": 0.6,
        }
        inner = {
            "tag_id": 1,
            "role": "inner",
            "distance_m": 0.53,
            "decision_margin": 48.0,
            "hamming": 2,
            "reprojection_error_px": 0.1,
        }
        selected = select_primary_tag(
            (outer, inner),
            previous_tag_id=1,
            switch_to_inner_below_m=0.35,
            hysteresis_m=0.05,
            quality_gates=gates,
            prefer_outer=True,
        )
        self.assertEqual(selected["tag_id"], 0)

        selected = select_primary_tag(
            (dict(outer, hamming=3), inner),
            previous_tag_id=0,
            switch_to_inner_below_m=0.35,
            hysteresis_m=0.05,
            quality_gates=gates,
            prefer_outer=True,
        )
        self.assertEqual(selected["tag_id"], 1)

        selected = select_primary_tag(
            (inner,),
            previous_tag_id=0,
            switch_to_inner_below_m=0.35,
            hysteresis_m=0.05,
            quality_gates=gates,
            prefer_outer=True,
        )
        self.assertEqual(selected["tag_id"], 1)


if __name__ == "__main__":
    unittest.main()
