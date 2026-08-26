import json
import math
import unittest
from pathlib import Path

from air_ground_landing.hybrid_guidance import (
    ControlOwner,
    ElasticTrackerStatus,
    HybridGuidanceConfig,
    HybridGuidanceCoordinator,
    HybridGuidanceInputs,
    IbvsConfig,
    IbvsFeatureController,
    IbvsMode,
)
from air_ground_landing.landing_target_bridge import (
    MAV_FRAME_BODY_FRD,
    BridgeConfig,
    LandingTargetBridge,
)
from air_ground_landing.models import MovingPadEstimate, UavState, UgvState
from air_ground_landing.moving_landing_supervisor import (
    LandingState,
    MovingLandingSupervisor,
    SupervisorConfig,
    SupervisorDecision,
    SupervisorInputs,
)
from air_ground_landing.moving_pad_estimator import EstimatorConfig, MovingPadEstimator
from air_ground_landing.stack_replay import MovingLandingReplay


HERE = Path(__file__).resolve().parents[1]
CONFIG_PATH = HERE / "config" / "moving_landing.prototype.json"


def load_config():
    return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))


def vision_status(
    sequence=1,
    *,
    age_ms=20.0,
    margin=60.0,
    reprojection=0.4,
    tag_id=0,
    hamming=0,
):
    x_m, y_m, z_m = 0.10, -0.05, 0.65
    tag_size_m = 0.1 if tag_id == 0 else 0.02
    return {
        "sensor": "ov9281",
        "mode": "apriltag",
        "found": True,
        "analysis_sequence": sequence,
        "analysis_size": [1280, 800],
        "pixel_source": "Y_MONO",
        "tag_family": "tag36h11",
        "tag_size_m": tag_size_m,
        "tag_id": tag_id,
        "frame_age_ms": age_ms,
        "x_m": x_m,
        "y_m": y_m,
        "z_m": z_m,
        "distance_m": math.sqrt(x_m * x_m + y_m * y_m + z_m * z_m),
        "decision_margin": margin,
        "hamming": hamming,
        "reprojection_error_px": reprojection,
        "flight_controller_connected": False,
    }


def uav(timestamp, *, z=-1.0, speed=0.1, mode="LOITER", armed=True, landed=False):
    return UavState(
        timestamp_s=timestamp,
        position_ned_m=(0.0, 0.0, z),
        velocity_ned_mps=(speed, 0.0, 0.0),
        quaternion_body_to_ned=(1.0, 0.0, 0.0, 0.0),
        mode=mode,
        armed=armed,
        landed=landed,
        link_healthy=True,
        velocity_source_independent_of_deck=False,
    )


def ugv(timestamp, *, speed=0.1):
    return UgvState(
        timestamp_s=timestamp,
        position_ned_m=(0.0, 0.0, 0.0),
        velocity_ned_mps=(speed, 0.0, 0.0),
        healthy=True,
        emergency_stop=False,
        common_origin_valid=True,
    )


def pad(timestamp, *, x=0.05, z=0.0, speed=0.1, vision_age=0.05, ugv_age=0.0):
    sources = tuple(
        source
        for source, age in (("APRILTAG", vision_age), ("UGV_ODOMETRY", ugv_age))
        if age is not None and age <= 0.5
    )
    return MovingPadEstimate(
        timestamp_s=timestamp,
        position_ned_m=(x, 0.0, z),
        velocity_ned_mps=(speed, 0.0, 0.0),
        covariance_m2=(0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.02),
        quality=0.9,
        sources=sources,
        vision_age_s=vision_age,
        ugv_age_s=ugv_age,
    )


def supervisor_decision(state, *, descent=False, publish=True):
    return SupervisorDecision(
        state=state,
        reason="TEST",
        publish_landing_target=publish,
        request_land_mode=descent,
        request_hold_mode=False,
        request_ugv_stop=False,
        descent_authorized=descent,
        abort_action=None,
        horizontal_error_m=0.02,
        relative_speed_mps=0.01,
        height_above_pad_m=0.5,
    )


class LandingTargetBridgeTests(unittest.TestCase):
    def test_quality_gate_and_body_frd_packet(self):
        bridge = LandingTargetBridge(BridgeConfig.from_mapping(load_config()))
        result = bridge.process_status(
            vision_status(), received_time_s=10.0, wall_time_usec=10_000_000
        )
        self.assertTrue(result.accepted)
        self.assertIsNotNone(result.packet)
        self.assertEqual(result.packet.frame, MAV_FRAME_BODY_FRD)
        self.assertEqual(result.packet.position_valid, 1)
        self.assertAlmostEqual(result.observation.position_body_frd_m[0], -0.10)
        self.assertAlmostEqual(result.observation.position_body_frd_m[1], 0.05)
        self.assertAlmostEqual(result.observation.position_body_frd_m[2], 0.65)

        stale = bridge.process_status(
            vision_status(2, age_ms=300.0),
            received_time_s=10.1,
            wall_time_usec=10_100_000,
        )
        self.assertFalse(stale.accepted)
        self.assertEqual(stale.reason, "STALE_FRAME")
        self.assertTrue(bridge.target_lost(10.6))

    def test_per_tag_quality_gates_allow_two_bit_nested_tags(self):
        config = BridgeConfig.from_mapping(load_config())

        accepted_outer = LandingTargetBridge(config).process_status(
            vision_status(hamming=2, margin=55.0, reprojection=0.8),
            received_time_s=10.0,
            wall_time_usec=10_000_000,
        )
        self.assertTrue(accepted_outer.accepted)
        self.assertEqual(accepted_outer.observation.tag_id, 0)

        outer_hamming_high = LandingTargetBridge(config).process_status(
            vision_status(hamming=3, margin=55.0, reprojection=0.8),
            received_time_s=10.0,
            wall_time_usec=10_000_000,
        )
        self.assertEqual(outer_hamming_high.reason, "HAMMING_LIMIT")

        outer_margin_low = LandingTargetBridge(config).process_status(
            vision_status(hamming=2, margin=49.0, reprojection=0.8),
            received_time_s=10.0,
            wall_time_usec=10_000_000,
        )
        self.assertEqual(outer_margin_low.reason, "LOW_DECISION_MARGIN")

        outer_reprojection_high = LandingTargetBridge(config).process_status(
            vision_status(hamming=2, margin=55.0, reprojection=1.1),
            received_time_s=10.0,
            wall_time_usec=10_000_000,
        )
        self.assertEqual(
            outer_reprojection_high.reason,
            "REPROJECTION_ERROR_LIMIT",
        )

        accepted_inner = LandingTargetBridge(config).process_status(
            vision_status(
                tag_id=1,
                hamming=2,
                margin=40.0,
                reprojection=1.0,
            ),
            received_time_s=10.0,
            wall_time_usec=10_000_000,
        )
        self.assertTrue(accepted_inner.accepted)
        self.assertEqual(accepted_inner.observation.tag_id, 1)

        inner_hamming_high = LandingTargetBridge(config).process_status(
            vision_status(
                tag_id=1,
                hamming=3,
                margin=40.0,
                reprojection=1.0,
            ),
            received_time_s=10.0,
            wall_time_usec=10_000_000,
        )
        self.assertEqual(inner_hamming_high.reason, "HAMMING_LIMIT")

        unsupported = vision_status(tag_id=9)
        unsupported["tag_size_m"] = 0.1
        unsupported_result = LandingTargetBridge(config).process_status(
            unsupported,
            received_time_s=10.0,
            wall_time_usec=10_000_000,
        )
        self.assertEqual(unsupported_result.reason, "TAG_ID_NOT_CONFIGURED")


class MovingPadEstimatorTests(unittest.TestCase):
    def test_fuses_body_observation_and_aligned_ugv_odometry(self):
        bridge = LandingTargetBridge(BridgeConfig.from_mapping(load_config()))
        observation = bridge.process_status(
            vision_status(), received_time_s=10.0, wall_time_usec=10_000_000
        ).observation
        estimator = MovingPadEstimator(EstimatorConfig.from_mapping(load_config()))
        aircraft = UavState(
            timestamp_s=observation.capture_time_s,
            position_ned_m=(1.0, 2.0, -0.65),
            velocity_ned_mps=(0.1, 0.0, 0.0),
            quaternion_body_to_ned=(1.0, 0.0, 0.0, 0.0),
            link_healthy=True,
        )
        estimator.update_vision(observation, aircraft)
        estimator.update_ugv(UgvState(
            timestamp_s=10.0,
            position_ned_m=(0.9, 2.05, 0.0),
            velocity_ned_mps=(0.1, 0.0, 0.0),
            healthy=True,
            common_origin_valid=True,
        ))
        estimate = estimator.estimate(10.1)
        self.assertIsNotNone(estimate)
        self.assertEqual(set(estimate.sources), {"APRILTAG", "UGV_ODOMETRY"})
        self.assertGreater(estimate.quality, 0.5)
        self.assertAlmostEqual(estimate.velocity_ned_mps[0], 0.1, places=2)
        self.assertLess(abs(estimate.position_ned_m[0] - 0.91), 0.08)


class HybridGuidanceTests(unittest.TestCase):
    def test_ibvs_features_and_single_writer_handover(self):
        config = load_config()
        config["hybrid_guidance"]["arbitration"]["ibvs_alignment_hold_s"] = 0.0
        bridge = LandingTargetBridge(BridgeConfig.from_mapping(config))
        camera = config["hybrid_guidance"]["camera"]
        cx, cy = camera["cx_px"], camera["cy_px"]
        status = vision_status(sequence=77)
        status["overlay_points"] = [
            [cx - 50.0, cy - 50.0],
            [cx + 50.0, cy - 50.0],
            [cx + 50.0, cy + 50.0],
            [cx - 50.0, cy + 50.0],
        ]
        observation = bridge.process_status(
            status,
            received_time_s=10.0,
            wall_time_usec=10_000_000,
        ).observation
        ibvs_config = IbvsConfig.from_mapping(config)
        feature_controller = IbvsFeatureController(ibvs_config)
        features = feature_controller.process_status(
            status,
            observation,
            now_s=10.0,
        )
        self.assertTrue(features.valid)
        self.assertTrue(features.aligned)
        self.assertEqual(features.mode, IbvsMode.IBVS_4DOF)
        self.assertLess(features.centroid_error_px, 1.0e-6)
        self.assertFalse(features.final_ready)

        elastic = ElasticTrackerStatus(
            timestamp_s=10.0,
            heartbeat_healthy=True,
            map_fresh=True,
            target_prediction_fresh=True,
            trajectory_valid=True,
            visibility_corridor_valid=True,
            trajectory_id=7,
        )
        coordinator = HybridGuidanceCoordinator(
            HybridGuidanceConfig.from_mapping(config),
            ibvs_config,
        )

        def decide(state, *, descent=False):
            return coordinator.decide(HybridGuidanceInputs(
                timestamp_s=10.0,
                supervisor=supervisor_decision(state, descent=descent),
                uav=uav(10.0),
                pad=pad(10.0),
                elastic=elastic,
                ibvs=features,
            ))

        rendezvous = decide(LandingState.RENDEZVOUS)
        match = decide(LandingState.MATCH_VELOCITY)
        descend = decide(LandingState.DESCEND, descent=True)
        final = decide(LandingState.FINAL_APPROACH, descent=True)
        self.assertEqual(rendezvous.control_owner, ControlOwner.ELASTIC_GUIDED)
        self.assertEqual(match.control_owner, ControlOwner.IBVS_GUIDED)
        self.assertEqual(descend.control_owner, ControlOwner.AC_PRECLAND_LAND)
        self.assertEqual(final.control_owner, ControlOwner.HOLD)
        self.assertIsNotNone(match.requested_body_velocity_frd_mps)
        self.assertLessEqual(
            math.hypot(*match.requested_body_velocity_frd_mps[:2]),
            config["hybrid_guidance"]["arbitration"]["maximum_total_horizontal_speed_mps"],
        )
        for decision in (rendezvous, match, descend, final):
            writers = sum((
                decision.elastic_trajectory_authorized,
                decision.ibvs_velocity_authorized,
                decision.ac_precland_authorized,
            ))
            self.assertLessEqual(writers, 1)


class MovingLandingSupervisorTests(unittest.TestCase):
    def test_safe_sequence_stops_ugv_before_final_touchdown(self):
        supervisor = MovingLandingSupervisor(SupervisorConfig.from_mapping(load_config()))

        def step(
            t,
            *,
            aircraft=None,
            rover=None,
            estimate=None,
            target_age=0.05,
            rng=1.0,
            contact=False,
            descent=False,
        ):
            return supervisor.step(SupervisorInputs(
                timestamp_s=t,
                mission_enabled=True,
                operator_authorized=True,
                pilot_override=False,
                descent_requested=descent,
                uav=aircraft or uav(t),
                ugv=rover or ugv(t),
                pad=estimate or pad(t),
                landing_target_age_s=target_age,
                rangefinder_distance_m=rng,
                contact_confirmed=contact,
            ))

        self.assertEqual(step(0.0).state, LandingState.RENDEZVOUS)
        self.assertEqual(step(0.1).state, LandingState.TRACK_PAD)
        self.assertEqual(step(0.2).state, LandingState.MATCH_VELOCITY)
        self.assertEqual(step(0.3).state, LandingState.MATCH_VELOCITY)
        waiting = step(0.95)
        self.assertEqual(waiting.state, LandingState.MATCH_VELOCITY)
        self.assertEqual(waiting.reason, "WAIT_SWD_DESCENT_REQUEST")
        descend = step(1.0, descent=True)
        self.assertEqual(descend.state, LandingState.DESCEND)
        self.assertTrue(descend.descent_authorized)
        self.assertTrue(descend.request_land_mode)

        final = step(
            1.05,
            aircraft=uav(1.05, z=-0.20, mode="LAND"),
            estimate=pad(1.05),
            rng=0.20,
            descent=True,
        )
        self.assertEqual(final.state, LandingState.FINAL_APPROACH)
        self.assertTrue(final.request_ugv_stop)
        self.assertFalse(final.descent_authorized)

        range_only = step(
            1.15,
            aircraft=uav(1.15, z=-0.14, speed=0.0, mode="LAND"),
            rover=ugv(1.15, speed=0.0),
            estimate=pad(1.15, speed=0.0, vision_age=1.0, ugv_age=0.0),
            target_age=1.0,
            rng=0.11,
            descent=True,
        )
        self.assertTrue(range_only.descent_authorized)

        touchdown = step(
            1.25,
            aircraft=uav(1.25, z=-0.04, speed=0.0, mode="LAND", armed=False, landed=True),
            rover=ugv(1.25, speed=0.0),
            estimate=pad(1.25, speed=0.0, vision_age=1.0, ugv_age=0.0),
            target_age=1.0,
            rng=0.11,
            contact=True,
            descent=True,
        )
        self.assertEqual(touchdown.state, LandingState.TOUCHDOWN)
        self.assertEqual(step(
            1.35,
            aircraft=uav(1.35, speed=0.0, mode="LAND", armed=False, landed=True),
            rover=ugv(1.35, speed=0.0),
            estimate=pad(1.35, speed=0.0),
        ).state, LandingState.COMPLETE)

    def test_swd_off_cancels_descent_and_returns_to_velocity_match(self):
        supervisor = MovingLandingSupervisor(SupervisorConfig.from_mapping(load_config()))
        for timestamp in (0.0, 0.1, 0.2, 0.3, 0.95):
            decision = supervisor.step(SupervisorInputs(
                timestamp_s=timestamp,
                mission_enabled=True,
                operator_authorized=True,
                pilot_override=False,
                descent_requested=True,
                uav=uav(timestamp),
                ugv=ugv(timestamp),
                pad=pad(timestamp),
                landing_target_age_s=0.05,
                rangefinder_distance_m=1.0,
            ))
        self.assertEqual(decision.state, LandingState.DESCEND)

        cancelled = supervisor.step(SupervisorInputs(
            timestamp_s=1.0,
            mission_enabled=True,
            operator_authorized=True,
            pilot_override=False,
            descent_requested=False,
            uav=uav(1.0),
            ugv=ugv(1.0),
            pad=pad(1.0),
            landing_target_age_s=0.05,
            rangefinder_distance_m=0.8,
        ))
        self.assertEqual(cancelled.state, LandingState.MATCH_VELOCITY)
        self.assertEqual(cancelled.reason, "SWD_DESCENT_CANCELLED_RESUME_FOLLOW")
        self.assertFalse(cancelled.request_land_mode)

    def test_target_loss_aborts_descent(self):
        config = SupervisorConfig.from_mapping(load_config())
        supervisor = MovingLandingSupervisor(config)
        times = (0.0, 0.1, 0.2, 0.3, 0.95)
        for timestamp in times:
            decision = supervisor.step(SupervisorInputs(
                timestamp_s=timestamp,
                mission_enabled=True,
                operator_authorized=True,
                pilot_override=False,
                descent_requested=True,
                uav=uav(timestamp),
                ugv=ugv(timestamp),
                pad=pad(timestamp),
                landing_target_age_s=0.05,
                rangefinder_distance_m=1.0,
            ))
        self.assertEqual(decision.state, LandingState.DESCEND)
        aborted = supervisor.step(SupervisorInputs(
            timestamp_s=1.1,
            mission_enabled=True,
            operator_authorized=True,
            pilot_override=False,
            descent_requested=True,
            uav=uav(1.1),
            ugv=ugv(1.1),
            pad=pad(1.1),
            landing_target_age_s=1.0,
            rangefinder_distance_m=0.8,
        ))
        self.assertEqual(aborted.state, LandingState.ABORT)
        self.assertEqual(aborted.abort_action, "HOLD_OR_STATIC_LAND")
        self.assertTrue(aborted.request_ugv_stop)


class IntegratedReplayTests(unittest.TestCase):
    def test_one_snapshot_runs_all_modules_without_transmission(self):
        replay = MovingLandingReplay(load_config())
        result = replay.process({
            "timestamp_s": 10.0,
            "wall_time_usec": 10_000_000,
            "mission_enabled": True,
            "operator_authorized": True,
            "vision_status": vision_status(),
            "uav": {
                "timestamp_s": 9.98,
                "position_ned_m": [0.0, 0.0, -0.65],
                "velocity_ned_mps": [0.1, 0.0, 0.0],
                "quaternion_body_to_ned": [1.0, 0.0, 0.0, 0.0],
                "mode": "LOITER",
                "armed": True,
                "landed": False,
                "link_healthy": True,
                "velocity_source_independent_of_deck": False
            },
            "ugv": {
                "position_ned_m": [-0.1, 0.05, 0.0],
                "velocity_ned_mps": [0.1, 0.0, 0.0],
                "healthy": True,
                "emergency_stop": False,
                "common_origin_valid": True
            },
            "rangefinder_distance_m": 0.65
        })
        self.assertTrue(result["bridge"]["accepted"])
        self.assertIsNotNone(result["pad_estimate"])
        self.assertEqual(result["supervisor"]["state"], "RENDEZVOUS")
        self.assertFalse(result["ibvs_features"]["valid"])
        self.assertEqual(result["hybrid_guidance"]["control_owner"], "HOLD")
        self.assertFalse(result["mavlink_transmitted"])
        self.assertFalse(result["vehicle_command_transmitted"])


if __name__ == "__main__":
    unittest.main()
